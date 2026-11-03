#!/usr/bin/env python3
"""Actual Gazebo views/poses for slot scenarios, including 13 obstacle contact probes."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as E
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from random_scenes import Validator, legal_obstacle
import generate as gen

def stop(proc):
    if proc.poll() is not None: return
    os.killpg(proc.pid,signal.SIGINT)
    try: proc.wait(timeout=12)
    except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGTERM); proc.wait(timeout=5)

def poses():
    for _ in range(8):
        try:
            q=subprocess.run(['gz','topic','-e','-t','/world/gongchuang/pose/info','-n','1','--json-output'],
                capture_output=True,text=True,timeout=5)
            return json.JSONDecoder().raw_decode(q.stdout.lstrip())[0]
        except (ValueError,subprocess.TimeoutExpired): pass
    raise RuntimeError('No Gazebo pose data')

def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True)
    c=yaml.safe_load((ROOT/'config/field_geometry.yaml').read_text())
    settings=yaml.safe_load((ROOT/'config/random_generation.yaml').read_text()); validator=Validator(c,settings)
    os.environ['GZ_PARTITION']=f'gongchuang_slots_acceptance_{os.getpid()}'
    os.environ['GC_SOFTWARE_RENDER']='1'
    results={}
    examples=json.loads((ROOT/'config/generated_slot_examples.json').read_text())
    for case,world,info in [
        ('default_2','audit_random',json.loads((ROOT/'config/generated_randomization.json').read_text())),
        *[(name,'audit_'+name,info) for name,info in examples.items()],
        ('blocked_slots','audit_central_blocked',json.loads((ROOT/'config/generated_presets.json').read_text())['central_blocked'])]:
        out=a.output/case; out.mkdir(exist_ok=True)
        tree=E.parse(ROOT/'worlds'/(world+'.sdf'))
        positions=[list(map(float,i.findtext('pose').split()[:2])) for i in tree.findall('.//include')
            if i.findtext('uri')=='model://cylinder_obstacle']
        assert positions==info['obstacle_positions_m']
        assert all(legal_obstacle(c,*pt) for pt in positions)
        with (out/'gazebo.log').open('w') as log:
            flags=[] if case=='slots_debug' else ['-s']
            proc=subprocess.Popen(['bash',str(ROOT/'scripts/start.sh'),world,*flags],stdout=log,stderr=log,start_new_session=True)
            try:
                views=['top','cross','qr_board'] if case=='default_2' else ['top']
                subprocess.run([sys.executable,str(ROOT/'scripts/capture.py'),'--output',str(out),'--views',*views],check=True,timeout=110)
                live=poses(); (out/'actual_poses.json').write_text(json.dumps(live,indent=2)+'\n')
                lookup={p['name']:p.get('position',{}) for p in live['pose']}
                obs={name:pt for name,pt in lookup.items() if name.startswith('obstacle_')}
                assert len(obs)==len(positions)
                for entry,pt in zip(info.get('obstacles',[]),positions):
                    actual=obs[entry['entity_name']]
                    assert abs(actual.get('x',0)-pt[0])<1e-8 and abs(actual.get('y',0)-pt[1])<1e-8
                    assert pt==settings['candidate_slots']['slots'][entry['slot_id']]['xy_m']
                report={'source':'actual Gazebo Harmonic camera and transport poses','world':world,
                    'selected_slots':info['selected_slots'],'count':len(positions),'seed':info['seed'],
                    'obstacle_positions_m':positions,'pose_tolerance_m':1e-8,'pose_match':True,
                    'navigation':validator.assess(positions)}
                (out/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
                results[case]={'count':len(positions),'slots':info['selected_slots'],
                    'status':report['navigation']['status'],'live_poses':'pass','screenshot':case+'/top.png'}
                print(case,results[case],flush=True)
            finally: stop(proc)
    # Every slot receives a real cylinder; drop a small dynamic probe onto every top face.
    os.environ['GZ_SIM_RESOURCE_PATH']=str(ROOT/'models')
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(ROOT.parents[1]/'install/gongchuang_gazebo_world/lib')
    slots=settings['candidate_slots']['slots']
    tree=gen.world(c,'slot_contact_probes',[e['xy_m'] for e in slots.values()],probe=True)
    world=tree.find('world'); world.remove(world.find("model[@name='contact_probe']"))
    for sid,e in list(slots.items())+[('control',{'xy_m':[.4,.4]})]:
        mo=gen.tag(world,'model',name='probe_'+sid); gen.tag(mo,'pose',[*e['xy_m'],.3,0,0,0])
        li=gen.tag(mo,'link',name='body'); inert=gen.tag(li,'inertial'); gen.tag(inert,'mass',.01)
        inertia=gen.tag(inert,'inertia')
        for key in ('ixx','iyy','izz'): gen.tag(inertia,key,.000001)
        gen.solid(li,'probe',[0,0,0,0,0,0],[.01,.02],[.8,.1,.1,1],'cylinder')
    probe_file=a.output/'slot_contact_probes.sdf'; gen.save_xml(tree,probe_file)
    with (a.output/'slot_contact_probes.log').open('w') as log:
        proc=subprocess.Popen(['gz','sim','-s','-r',str(probe_file)],stdout=log,stderr=log,start_new_session=True)
        try:
            poses(); time.sleep(3); live=poses()
            (a.output/'contact_actual_poses.json').write_text(json.dumps(live,indent=2)+'\n')
            lookup={p['name']:p.get('position',{}) for p in live['pose']}
            heights={}
            for sid in list(slots)+['control']:
                actual=lookup['probe_'+sid]; expected=.01 if sid=='control' else .11
                assert abs(actual.get('z',0)-expected)<.002,(sid,actual)
                heights[sid]=actual['z']
            results['physical_contacts']={'status':'pass','13_top_contact_heights_m':heights,'tolerance_m':.002}
            print('physical_contacts PASS',flush=True)
        finally: stop(proc)
    (a.output/'results.json').write_text(json.dumps(results,indent=2)+'\n')

if __name__=='__main__': main()
