#!/usr/bin/env python3
"""Cross-check an isolated seed bundle's certificate against live Gazebo entities."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
from random_scenes import verify_certificate, Validator

p=argparse.ArgumentParser(); p.add_argument('--bundle',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
bundle=a.bundle.resolve(); pkg=Path(__file__).resolve().parents[1]
c=yaml.safe_load((bundle/'config/field_geometry.yaml').read_text())
settings=yaml.safe_load((bundle/'config/random_generation.yaml').read_text())
info=json.loads((bundle/'config/generated_randomization.json').read_text())
cert=json.loads((bundle/'maps/random_seeded_certificate.json').read_text())
assessment=Validator(c,settings).assess(info['obstacle_positions_m'])
expected_status=info.get('reachability_status','PATH')
assert assessment['status']==expected_status
assert assessment['status']!='INVALID_PLACEMENT'
if expected_status=='PATH': assert verify_certificate(c,settings,info['obstacle_positions_m'],cert)
else: assert cert is None
os.environ['GZ_PARTITION']=f'gongchuang_bundle_validation_{os.getpid()}'
os.environ['GZ_SIM_RESOURCE_PATH']=str(bundle/'models'); os.environ['SDF_PATH']=str(bundle/'models')
os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(pkg.parents[1]/'install/gongchuang_gazebo_world/lib')
file=bundle/'worlds/random_seeded.sdf'
parse=subprocess.run(['gz','sdf','-k',str(file)],capture_output=True,text=True,timeout=20)
assert parse.returncode==0,parse.stdout+parse.stderr
with (a.output/'server.log').open('w') as log:
    proc=subprocess.Popen(['gz','sim','-s','-r',str(file)],stdout=log,stderr=log,start_new_session=True)
    try:
        live=None
        for _ in range(6):
            try:
                q=subprocess.run(['gz','topic','-e','-t','/world/gongchuang/pose/info','-n','1','--json-output'],capture_output=True,text=True,timeout=4)
                live=json.JSONDecoder().raw_decode(q.stdout.lstrip())[0]
                break
            except (subprocess.TimeoutExpired,ValueError): pass
        assert live is not None,'No live Gazebo pose data'
        (a.output/'actual_poses.json').write_text(json.dumps(live,indent=2)+'\n')
        lookup={p['name']:p.get('position',{}) for p in live['pose']}
        for i,point in enumerate(info['obstacle_positions_m'],1):
            actual=lookup[f'obstacle_{i}']; assert abs(actual['x']-point[0])<1e-8 and abs(actual['y']-point[1])<1e-8
        mission=json.loads((bundle/'config/generated_mission.json').read_text())
        assert mission['task_code']==info['task_code']
        for color in mission['batch1']['colors']:
            assert any(name.endswith(f'color{color}') for name in lookup if name.startswith('batch1_'))
        result={'seed':info['seed'],'count':info['obstacle_count'],'task_code':info['task_code'],
            'reachability_status':expected_status,'raw_to_coarse':assessment['raw_to_coarse'],
            'sdf':'pass','obstacle_live_poses':'pass','material_color_entities':'pass'}
        (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result),flush=True)
    finally:
        os.killpg(proc.pid,signal.SIGINT)
        try: proc.wait(timeout=12)
        except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGTERM); proc.wait(timeout=5)
