#!/usr/bin/env python3
"""R1..R13 lottery, robust A*, validation and artifact consistency regressions."""
import copy
import itertools
import json
import math
from collections import Counter
from pathlib import Path
import sys
import unittest
from unittest import mock
import xml.etree.ElementTree as E
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from random_scenes import (Validator, sample_scene, seeded_task, verify_certificate, legal_obstacle,
    preset_scene, select_scene, draw_ids, validate_slots, footprint_half, sweep_clear, forbidden)
C=yaml.safe_load((ROOT/'config/field_geometry.yaml').read_text())
SETTINGS=yaml.safe_load((ROOT/'config/random_generation.yaml').read_text())


class RandomScenes(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.validator=Validator(C,SETTINGS)

    def test_01_seed_reproducibility_and_independent_colors(self):
        a=select_scene(C,SETTINGS,42,validator=self.validator)
        self.assertEqual(a,select_scene(C,SETTINGS,42,validator=self.validator))
        self.assertNotEqual(set(a[1]['selected_slots']),set(draw_ids(SETTINGS,43)))
        self.assertEqual(a[1]['task_code'],seeded_task(42))

    def test_02_1000_seed_uniform_unfiltered_distribution(self):
        frequency=Counter(); combos=Counter(); colors=set()
        for seed in range(1000):
            ids=draw_ids(SETTINGS,seed)
            self.assertEqual(len(ids),2); self.assertEqual(len(ids),len(set(ids)))
            self.assertEqual(ids,draw_ids(SETTINGS,seed))
            frequency.update(ids); combos.update([tuple(sorted(ids))])
            groups=seeded_task(seed).split('+')
            self.assertEqual(sorted(groups[0]),sorted(groups[2]))
            self.assertEqual(len(set(groups[0])),3)
            for g in (groups[1],groups[3]): self.assertEqual(sorted(g),list('123'))
            colors.update(groups[0])
        self.assertEqual(colors,set('123456')); self.assertEqual(len(combos),78)
        expected=2000/13
        chi=sum((frequency[f'R{i}']-expected)**2/expected for i in range(1,14))
        self.assertLess(chi,32.91)
        self.assertEqual(sum(frequency.values()),2000)

    def test_03_blocked_operating_pose_rejected(self):
        self.assertIsNone(self.validator.certify([[1.2,2.1]]))
        altered=copy.deepcopy(SETTINGS); altered['navigation_task']['goal']='qr_board'
        self.assertIsNone(Validator(C,altered).certify([[2.15,1.2]]))

    def test_04_invalid_seed_count_and_ids(self):
        for seed,count in [(-1,4),(2**32,4),(1,-1),(1,14),(True,2)]:
            with self.assertRaises(ValueError): draw_ids(SETTINGS,seed,count)
        for ids in [['R2','R2'],['R0'],['R14']]:
            with self.assertRaises(ValueError): draw_ids(SETTINGS,1,mode='manual_slots',manual_slots=ids)
        with self.assertRaises(ValueError): draw_ids(SETTINGS,1,2,'manual_slots',['R1'])
        with self.assertRaises(ValueError): draw_ids(SETTINGS,1,2,'stress_slots')

    def test_05_forged_route_and_budgets_rejected(self):
        pts,_,cert=sample_scene(C,SETTINGS,2027,4,self.validator)
        self.assertTrue(verify_certificate(C,SETTINGS,pts,cert))
        corrupt=copy.deepcopy(cert); corrupt['path_m']=[cert['start_m'],[.7,.7],cert['goal_m']]
        self.assertFalse(verify_certificate(C,SETTINGS,pts,corrupt))
        corrupt=copy.deepcopy(cert); corrupt['safety_margin_m']=0
        self.assertFalse(verify_certificate(C,SETTINGS,pts,corrupt))
        self.assertFalse(verify_certificate(C,SETTINGS,pts+[[.7,.7]],cert))

    def test_06_complete_cylinder_not_center_only(self):
        for pt in [[1.01,.775],[2.09,2.25],[.16,1.2],[.01,.4],[2.39,.7]]:
            self.assertFalse(legal_obstacle(C,*pt))
        for pt in [[1.175,.75],[.30,.40],[1.05,.775]]: self.assertTrue(legal_obstacle(C,*pt))

    def test_07_legacy_quantity_presets_keep_count_not_solvability_filter(self):
        for count,name in enumerate(['obstacles_0','obstacles_1','obstacles_2','obstacles_3','stress_4','stress_5','stress_6']):
            points,info,cert,result=preset_scene(C,SETTINGS,2027,name,self.validator)
            self.assertEqual(len(points),count); self.assertTrue(all(legal_obstacle(C,*p) for p in points))
            self.assertIn(result['status'],('PATH','NO_PATH'))
            if cert: self.assertTrue(verify_certificate(C,SETTINGS,points,cert))
            if count>=4: self.assertEqual(info['classification'],'stress_test_not_official_quantity')
        self.assertEqual(SETTINGS['obstacle_count'],2)

    def test_08_blocked_fixture_preserved_and_real_disconnection(self):
        ids=SETTINGS['presets']['central_blocked']['slots']
        expected=[SETTINGS['candidate_slots']['slots'][sid]['xy_m'] for sid in ids]
        with mock.patch('random_scenes.sample_scene',side_effect=AssertionError('No solvable resampling')):
            points,info,cert,result=preset_scene(C,SETTINGS,2027,'central_blocked',self.validator)
        self.assertEqual(points,expected); self.assertEqual(info['selected_slots'],ids)
        self.assertIsNone(cert); self.assertEqual(result['status'],'NO_PATH')
        self.assertEqual(result['raw_to_coarse']['reason'],'DISCONNECTED')

    def test_09_partial_central_block_can_be_bypassed(self):
        pt=SETTINGS['candidate_slots']['slots']['R12']['xy_m']
        self.assertEqual(self.validator.route_status([pt],self.validator.goals['raw_material'],self.validator.goals['coarse_slot_2'])['status'],'PATH')

    def test_10_13_positions_reference_identity_and_geometry_anchors(self):
        report=validate_slots(C,SETTINGS); self.assertEqual(len(report),13)
        s=SETTINGS['candidate_slots']['slots']
        # Independent transcription of the supplied image topology, not numeric ordering.
        expected={'R1':('east_north'),'R2':('north_east'),'R3':('north_west'),
            'R4':('west_north'),'R5':('west_south'),'R6':('south_west'),
            'R7':('south_east'),'R8':('east_south'),'R9':('cross_east'),
            'R10':('cross_north'),'R11':('cross_west'),'R12':('cross_south'),'R13':('cross_center')}
        for sid,relation in expected.items(): self.assertEqual(s[sid]['relation'],relation)
        west=C['field']['yellow_lower_left'][0][0]+C['field']['yellow_size'][0]/2
        east=C['field']['yellow_lower_left'][1][0]+C['field']['yellow_size'][0]/2
        self.assertEqual(s['R9']['xy_m'],[east,1.2]); self.assertEqual(s['R10']['xy_m'],[1.2,east])
        self.assertEqual(s['R11']['xy_m'],[west,1.2]); self.assertEqual(s['R12']['xy_m'],[1.2,west])
        self.assertEqual(s['R13']['xy_m'],[1.2,1.2])
        self.assertFalse(SETTINGS['candidate_slots']['calibrated'])

    def test_11_invalid_slot_reported_never_relocated(self):
        altered=copy.deepcopy(SETTINGS); altered['candidate_slots']['slots']['R10']['xy_m']=[.7,.7]
        before=copy.deepcopy(altered)
        with self.assertRaisesRegex(ValueError,'R10'): select_scene(C,altered,1,validator=self.validator)
        self.assertEqual(altered,before)

    def test_12_all_modes_and_manual_example(self):
        for mode in ['random_slots','solvable_slots','blocked_slots','stress_slots','manual_slots']:
            kw={'manual_slots':['R2','R7','R10']} if mode=='manual_slots' else {}
            pts,info,cert,result=select_scene(C,SETTINGS,2027,mode=mode,validator=self.validator,**kw)
            self.assertEqual(len(pts),len(set(info['selected_slots'])))
            if mode=='solvable_slots': self.assertEqual(result['status'],'PATH')
            if mode=='blocked_slots': self.assertEqual(result['status'],'NO_PATH')
            if mode=='blocked_slots': self.assertEqual(result['task_result']['reason'],'DISCONNECTED')
            if mode=='stress_slots': self.assertTrue(3<=len(pts)<=6)
            if mode=='manual_slots': self.assertEqual(info['selected_slots'],kw['manual_slots'])
        self.assertEqual({len(draw_ids(SETTINGS,i,mode='stress_slots')) for i in range(100)},{3,4,5,6})

    def test_13_actual_sdf_raster_json_and_markers(self):
        info=json.loads((ROOT/'config/generated_randomization.json').read_text())
        world=E.parse(ROOT/'worlds/random_seeded.sdf')
        obs=[i for i in world.findall('.//include') if i.findtext('uri')=='model://cylinder_obstacle']
        semantic=(ROOT/'maps/random_seeded_semantic.pgm').read_bytes().split(b'\n',3)[3]
        records=json.loads((ROOT/'maps/random_seeded_obstacles.json').read_text())
        self.assertEqual(records['obstacles'],info['obstacles'])
        self.assertEqual(records['map_to_world'],[0]*6)
        selected=set(info['selected_slots'])
        for inc,entry in zip(obs,info['obstacles']):
            self.assertEqual(inc.findtext('name'),entry['entity_name'])
            self.assertEqual(list(map(float,inc.findtext('pose').split()[:3])),entry['position_m'])
            self.assertEqual(entry['position_m'][:2],SETTINGS['candidate_slots']['slots'][entry['slot_id']]['xy_m'])
        for sid,e in SETTINGS['candidate_slots']['slots'].items():
            x,y=e['xy_m']; idx=(479-int(y/.005))*480+int(x/.005)
            self.assertEqual(semantic[idx]==7,sid in selected,sid)
        debug=E.parse(ROOT/'models/slot_debug/model.sdf')
        self.assertEqual(len(debug.findall('.//collision')),0)
        self.assertEqual(len(debug.findall('.//visual')),26)
        self.assertIsNone(world.find(".//include[name='candidate_slot_debug']"))

    def test_14_error_budgets_and_full_sweep(self):
        v=SETTINGS['validation']
        self.assertEqual(v['robot_square_side'],.3); self.assertEqual(v['safety_margin'],.01)
        self.assertEqual(v['tracking_error'],.01); self.assertEqual(v['heading_error_deg'],3)
        half=footprint_half(SETTINGS)
        for deg in range(-300,301):
            angle=math.radians(deg/100)
            self.assertLessEqual(.15*(abs(math.cos(angle))+abs(math.sin(angle)))+.02,half+1e-10)
        self.assertFalse(sweep_clear([1.2,.4],[1.2,2.1],half,C,[[1.2,1.2]],forbidden(C)))
        self.assertFalse(sweep_clear([.8,.8],[.8,.8],half,C,[],forbidden(C)))
        self.assertEqual(self.validator.route_status([],[-.1,1.2],[1.2,.4])['status'],'NO_PATH')

    def test_15_random_retains_blocked_draws_and_all_slots_reachable_by_lottery(self):
        blocked=None
        for seed in range(100):
            points,info,cert,result=select_scene(C,SETTINGS,seed,6,validator=self.validator)
            if result['status']=='NO_PATH': blocked=info; break
        self.assertIsNotNone(blocked); self.assertEqual(blocked['attempts'],1)
        ids=draw_ids(SETTINGS,42,13); self.assertEqual(set(ids),set(SETTINGS['candidate_slots']['slots']))

    def test_16_all_example_maps_and_sdf_use_identical_slot_ids(self):
        cases=json.loads((ROOT/'config/generated_slot_examples.json').read_text())
        cases.update(json.loads((ROOT/'config/generated_presets.json').read_text()))
        for name,info in cases.items():
            world=E.parse(ROOT/'worlds'/(name+'.sdf'))
            pts=[list(map(float,i.findtext('pose').split()[:2])) for i in world.findall('.//include')
                if i.findtext('uri')=='model://cylinder_obstacle']
            self.assertEqual(pts,info['obstacle_positions_m'],name)
            self.assertEqual(pts,[SETTINGS['candidate_slots']['slots'][sid]['xy_m'] for sid in info['selected_slots']],name)
            data=(ROOT/'maps'/(name+'_semantic.pgm')).read_bytes().split(b'\n',3)[3]
            for sid,e in SETTINGS['candidate_slots']['slots'].items():
                x,y=e['xy_m']; idx=(479-int(y/.005))*480+int(x/.005)
                self.assertEqual(data[idx]==7,sid in info['selected_slots'],(name,sid))


if __name__=='__main__': unittest.main(verbosity=2)
