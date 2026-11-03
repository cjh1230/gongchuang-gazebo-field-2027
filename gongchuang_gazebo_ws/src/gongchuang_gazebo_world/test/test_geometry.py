#!/usr/bin/env python3
"""Independent assertions of official dimensions, SDF geometry, and raster alignment."""
import hashlib
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as E
import yaml

ROOT = Path(__file__).resolve().parents[1]
C = yaml.safe_load((ROOT/'config/field_geometry.yaml').read_text(encoding='utf-8'))
LAYOUTS = yaml.safe_load((ROOT/'config/obstacle_layouts.yaml').read_text(encoding='utf-8'))['layouts']
SLOTS = yaml.safe_load((ROOT/'config/random_generation.yaml').read_text())['candidate_slots']['slots']
LAYOUTS = {name:[SLOTS[sid]['xy_m'] for sid in ids] for name,ids in LAYOUTS.items()}


def model(name):
    return E.parse(ROOT/'models'/name/'model.sdf').getroot()


def nums(e, path):
    return [float(x) for x in e.findtext(path).split()]


def pgm(path):
    magic, dims, maxval, data = path.read_bytes().split(b'\n',3)
    return tuple(map(int,dims.split())), data


def pixel(data,x,y):
    r=C['map']['resolution']; n=round(2.4/r)
    return data[(n-1-int(y/r))*n+int(x/r)]


class Geometry(unittest.TestCase):
    def test_01_official_dimensions_independent_constants(self):
        f=C['field']
        self.assertEqual(f['size'],[2.4,2.4])
        self.assertEqual(f['central_lane_width'],.4)
        self.assertEqual(f['yellow_size'],[.45,.45])
        self.assertEqual(f['start_size'],[.3,.3])
        self.assertEqual(C['obstacle']['diameter'],.05)
        self.assertEqual(C['obstacle']['height'],.1)
        self.assertEqual(C['stations']['raw']['diameter'],.3)
        self.assertTrue(.08<=C['stations']['raw']['total_height']<=.1)
        for k in ['coarse','temporary']:
            self.assertEqual(C['stations'][k]['size'],[.58,.15])

    def test_02_ground_and_visual_collision_separation(self):
        m=model('competition_field')
        col=m.findall('.//collision')
        self.assertEqual(len(col),1)
        self.assertEqual(nums(col[0],'geometry/box/size')[:2],[2.4,2.4])
        z=nums(col[0],'pose')[2]; h=nums(col[0],'geometry/box/size')[2]
        self.assertAlmostEqual(z+h/2,0)
        ys=m.findall(".//visual[@name='yellow_1']")
        self.assertEqual(len(m.findall('.//visual')),7)
        self.assertEqual(nums(ys[0],'geometry/box/size')[:2],[.45,.45])

    def test_03_cross_width_derived_from_edges(self):
        m=model('competition_field')
        lo=m.find(".//visual[@name='yellow_1']")
        hi=m.find(".//visual[@name='yellow_2']")
        xlo=nums(lo,'pose')[0]+nums(lo,'geometry/box/size')[0]/2
        xhi=nums(hi,'pose')[0]-nums(hi,'geometry/box/size')[0]/2
        self.assertAlmostEqual(xhi-xlo,.4)
        hi=m.find(".//visual[@name='yellow_3']")
        self.assertAlmostEqual(nums(hi,'pose')[1]-nums(lo,'pose')[1]-.45,.4)

    def test_04_cylinders_contact_and_size(self):
        m=model('cylinder_obstacle')
        c=m.find('.//collision')
        self.assertEqual(float(c.findtext('geometry/cylinder/radius')),.025)
        h=float(c.findtext('geometry/cylinder/length'))
        self.assertEqual(h,.1)
        self.assertAlmostEqual(nums(c,'pose')[2]-h/2,0)
        self.assertEqual(m.findtext('model/static'),'true')

    def test_05_station_support(self):
        m=model('raw_material_station')
        heights=[]
        for link in m.findall('.//link'):
            lz=float(link.findtext('pose','0 0 0 0 0 0').split()[2])
            for col in link.findall('collision'):
                z=lz+nums(col,'pose')[2]; h=float(col.findtext('geometry/cylinder/length'))
                heights.append((z-h/2,z+h/2))
        heights.sort()
        self.assertAlmostEqual(heights[0][0],0)
        self.assertAlmostEqual(heights[0][1],heights[1][0])
        self.assertAlmostEqual(heights[1][1],C['stations']['raw']['total_height'])
        self.assertFalse(C['stations']['fine']['enabled'])
        self.assertIsNone(C['stations']['finished']['center'])

    def test_06_world_instances_no_illegal_overlaps(self):
        for name,points in LAYOUTS.items():
            w=E.parse(ROOT/'worlds'/(name+'.sdf')).getroot()
            obs=[i for i in w.findall('.//include') if i.findtext('uri')=='model://cylinder_obstacle']
            self.assertEqual(len(obs),len(points))
            for inc,pt in zip(obs,points):
                p=nums(inc,'pose'); self.assertEqual(p[:2],pt); self.assertEqual(p[2],0)
                x,y=pt; r=.025
                self.assertTrue(r<=x<=2.4-r and r<=y<=2.4-r)
                for a,b in C['field']['yellow_lower_left']:
                    dx=max(a-x,0,x-a-.45); dy=max(b-y,0,y-b-.45)
                    self.assertGreater(math.hypot(dx,dy),r)
            for i,p in enumerate(points):
                for q in points[i+1:]: self.assertGreater(math.dist(p,q),.05)
            for st in ['coarse','temporary','raw_material']:
                self.assertIsNotNone(w.find(f".//include[name='{st}']"))

    def test_07_map_semantics_and_selected_start(self):
        for name in LAYOUTS:
            dims,data=pgm(ROOT/'maps'/(name+'_transit.pgm'))
            self.assertEqual(dims,(480,480)); self.assertEqual(len(data),480**2)
            self.assertEqual(pixel(data,.7,.7),0)   # yellow
            self.assertEqual(pixel(data,2.25,2.25),0)
            self.assertEqual(pixel(data,1.2,2.35),205)
            for selected,free,blocked in [('start_1',(2.25,2.25),(2.25,.15)),('start_2',(2.25,.15),(2.25,2.25))]:
                _,d=pgm(ROOT/'maps'/(name+'_'+selected+'.pgm'))
                self.assertEqual(pixel(d,*free),254); self.assertEqual(pixel(d,*blocked),0)
        _,d=pgm(ROOT/'maps/empty_obstacles_transit.pgm')
        self.assertEqual(pixel(d,1.2,1.2),254)
        _,d=pgm(ROOT/'maps/single_obstacle_transit.pgm')
        self.assertEqual(pixel(d,1.2,1.2),0)
        _,d=pgm(ROOT/'maps/empty_obstacles_physical_nominal.pgm')
        self.assertEqual(pixel(d,.7,.7),254)  # physically flat yellow, rule forbidden
        self.assertEqual(pixel(d,2.25,2.25),254)

    def test_08_every_cell_matches_sdf_footprints(self):
        # Extract actual SDF instances, rather than regenerating expected positions from config.
        field=model('competition_field'); visuals=field.findall('.//visual')[1:]
        boxes=[]
        for v in visuals:
            x,y=nums(v,'pose')[:2]; a,b=nums(v,'geometry/box/size')[:2]
            boxes.append((x-a/2,y-b/2,x+a/2,y+b/2,v.attrib['name']))
        for layout in LAYOUTS:
            _,semantic=pgm(ROOT/'maps'/(layout+'_semantic.pgm'))
            n=480; res=.005
            for row in range(n):
                y=(n-row-.5)*res
                for col in range(n):
                    x=(col+.5)*res; value=semantic[row*n+col]
                    for a,b,d,e,name in boxes:
                        if a<=x<d and b<=y<e:
                            self.assertEqual(value,2 if name.startswith('yellow') else 3 if name=='start_1' else 4)

    def test_09_corridor_blocked_for_300mm_footprint(self):
        # At y=.925, yellow edges leave x in [1,1.4]. A 300mm square center fits
        # only in [1.15,1.25], which the center cylinder excludes for every heading.
        pts=LAYOUTS['blocked_corridor']; self.assertIn([1.2,.775],pts)
        low,high=1+.15,1.4-.15
        self.assertLessEqual(high-low,2*(.15+.025))

    def test_10_no_stale_generated_files(self):
        hashes=json.loads((ROOT/'generated_manifest.json').read_text())
        for name,expected in hashes.items():
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),expected,name)

    def test_11_numbered_six_ring_dimensions(self):
        m=model('processing_station')
        for slot in [1,2,3]:
            self.assertEqual(len(m.findall(f".//visual[@name='slot_{slot}_number']")),1)
            for index,diam in enumerate([.053,.058,.065,.075,.085,.095],1):
                v=m.find(f".//visual[@name='slot_{slot}_ring_{index}']")
                self.assertIsNotNone(v)
                uri=v.findtext('geometry/mesh/uri').replace('model://','')
                vertices=[]
                for line in (ROOT/'models'/uri).read_text().splitlines():
                    if line.startswith('vertex '): vertices.append(list(map(float,line.split()[1:])))
                radii=[math.hypot(x,y) for x,y,z in vertices]
                self.assertAlmostEqual(max(radii)*2,diam,places=9)
                self.assertAlmostEqual(max(radii)-min(radii),.0015,places=9)
        positions=[nums(m.find(f".//visual[@name='slot_{i}_number']"),'pose')[0] for i in [1,2,3]]
        self.assertEqual(positions,[.15,0.,-.15])

    def test_12_material_dimensions_mass_and_contact_shape(self):
        for color in range(1,7):
            m=model(f'material_{color}'); self.assertEqual(m.findtext('model/static'),'false')
            self.assertLessEqual(float(m.findtext('.//inertial/mass')),.1)
            bottom=m.find(".//collision[@name='bottom_flange_collision']")
            top=m.find(".//collision[@name='top_flange_collision']")
            self.assertEqual(float(bottom.findtext('geometry/cylinder/radius')),.025)
            self.assertEqual(float(top.findtext('geometry/cylinder/radius')),.015)
            lower=nums(bottom,'pose')[2]-.004; upper=nums(top,'pose')[2]+.004
            self.assertAlmostEqual(upper-lower,.06)
            self.assertEqual(len(m.findall('.//collision')),3)

    def test_13_raw_layout_120_degrees(self):
        w=E.parse(ROOT/'worlds/competition.sdf').getroot()
        poses=[nums(i,'pose') for i in w.findall('.//include') if (i.findtext('name') or '').startswith('batch1_')]
        self.assertEqual(len(poses),3)
        angles=[]
        for p in poses:
            dx,dy=p[0]-1.2,p[1]-2.47
            self.assertAlmostEqual(math.hypot(dx,dy),.1)
            self.assertAlmostEqual(p[2]-.03,.09+.0003)
            angles.append(math.atan2(dy,dx)%(2*math.pi))
        angles.sort()
        for i in range(3): self.assertAlmostEqual((angles[(i+1)%3]-angles[i])%(2*math.pi),2*math.pi/3)

    def test_14_qr_board_dimensions_and_occupancy(self):
        m=model('qr_board')
        self.assertEqual(nums(m.find('.//collision'),'geometry/box/size'),[.003,.297,.21])
        self.assertEqual(C['qr_board']['code_size'],.08)
        _,data=pgm(ROOT/'maps/competition_transit.pgm')
        self.assertEqual(pixel(data,2.399,1.2),0)
        self.assertEqual(pixel(data,2.38,1.2),254)

    def test_15_two_batches_color_consistency_and_stack(self):
        mission=json.loads((ROOT/'config/generated_mission.json').read_text())
        mapping=dict(zip(mission['batch1']['colors'],mission['batch1']['temporary_slots']))
        self.assertEqual(mission['batch2']['temporary_slots'],[mapping[color] for color in mission['batch2']['colors']])
        w=E.parse(ROOT/'worlds/stacking_demo.sdf').getroot()
        items=[i for i in w.findall('.//include') if (i.findtext('name') or '').startswith('batch')]
        self.assertEqual(len(items),6)
        for color in mission['batch1']['colors']:
            pair=[nums(i,'pose') for i in items if i.findtext('uri')==f'model://material_{color}']
            self.assertEqual(len(pair),2); self.assertEqual(pair[0][:2],pair[1][:2])
            self.assertAlmostEqual(abs(pair[0][2]-pair[1][2]),.06)
        for stage in ['batch2_ready','batch2_coarse']:
            w=E.parse(ROOT/'worlds'/(stage+'.sdf')).getroot()
            self.assertEqual(len([i for i in w.findall('.//include') if (i.findtext('name') or '').startswith('batch')]),6)

    def test_16_random_sdf_matches_certificate_and_colors(self):
        import sys
        sys.path.insert(0,str(ROOT/'scripts'))
        from random_scenes import verify_certificate, Validator
        info=json.loads((ROOT/'config/generated_randomization.json').read_text())
        cert=json.loads((ROOT/'maps/random_seeded_certificate.json').read_text())
        w=E.parse(ROOT/'worlds/random_seeded.sdf').getroot()
        points=[nums(i,'pose')[:2] for i in w.findall('.//include') if i.findtext('uri')=='model://cylinder_obstacle']
        self.assertEqual(len(points),info['obstacle_count'])
        settings=yaml.safe_load((ROOT/'config/random_generation.yaml').read_text())
        if info['reachability_status']=='NO_PATH':
            self.assertIsNone(cert)
            self.assertEqual(Validator(C,settings).assess(points)['status'],'NO_PATH')
            self.assertEqual(points,info['obstacle_positions_m'])
        else:
            self.assertTrue(verify_certificate(C,settings,points,cert))
        mission=json.loads((ROOT/'config/generated_mission.json').read_text())
        self.assertEqual(mission['task_code'],info['task_code'])
        colors=[int(i.findtext('uri').split('_')[-1]) for i in w.findall('.//include') if (i.findtext('name') or '').startswith('batch1_')]
        self.assertEqual(colors,mission['batch1']['colors'])


if __name__=='__main__': unittest.main(verbosity=2)
