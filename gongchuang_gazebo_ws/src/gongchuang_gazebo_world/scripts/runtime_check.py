#!/usr/bin/env python3
"""Actual Gazebo parse + bounded server runs for all test layouts and a gravity probe."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time
import yaml

ROOT=Path(__file__).resolve().parents[1]
if not (ROOT/'config').is_dir():
    from ament_index_python.packages import get_package_share_directory
    ROOT=Path(get_package_share_directory('gongchuang_gazebo_world'))


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
    os.environ['GZ_SIM_RESOURCE_PATH']=str(ROOT/'models')
    os.environ['SDF_PATH']=str(ROOT/'models')
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(ROOT.parents[1]/'install/gongchuang_gazebo_world/lib')
    os.environ['GZ_PARTITION']='gongchuang_validation'
    summary={}
    cfg=yaml.safe_load((ROOT/'config/field_geometry.yaml').read_text())
    layouts=yaml.safe_load((ROOT/'config/obstacle_layouts.yaml').read_text())['layouts']
    slots=yaml.safe_load((ROOT/'config/random_generation.yaml').read_text())['candidate_slots']['slots']
    layouts={name:[slots[sid]['xy_m'] for sid in ids] for name,ids in layouts.items()}
    randomization=json.loads((ROOT/'config/generated_randomization.json').read_text())
    presets=json.loads((ROOT/'config/generated_presets.json').read_text())
    layouts.update({name:info['obstacle_positions_m'] for name,info in presets.items()})
    for name in ['random_seeded','random_batch2_ready','random_stacking_demo']: layouts[name]=randomization['obstacle_positions_m']
    for name in ['empty_obstacles','single_obstacle','competition','blocked_corridor','contact_test','random_seeded','random_batch2_ready','random_stacking_demo','central_blocked','stress_6']:
        file=ROOT/'worlds'/(name+'.sdf')
        parse=subprocess.run(['gz','sdf','-k',str(file)],capture_output=True,text=True,timeout=30)
        (a.output/(name+'_parse.log')).write_text(parse.stdout+parse.stderr)
        if parse.returncode: raise RuntimeError('SDF parse failure: '+name)
        with (a.output/(name+'_server.log')).open('w') as log:
            proc=subprocess.Popen(['gz','sim','-s','-r','-v','3',str(file)],stdout=log,stderr=log,
                start_new_session=True)
            try:
                stats=None
                for attempt in range(8):
                    try:
                        q=subprocess.run(['gz','topic','-e','-t','/world/gongchuang/stats','-n','1'],
                            capture_output=True,text=True,timeout=4)
                        if 'iterations:' in q.stdout:
                            stats=q.stdout; break
                    except subprocess.TimeoutExpired: pass
                    if proc.poll() is not None: raise RuntimeError('Server stopped: '+name)
                if stats is None: raise RuntimeError('No simulation stats: '+name)
                # Let gravity settle while the bounded run remains alive.
                time.sleep(2)
                q=subprocess.run(['gz','topic','-e','-t','/world/gongchuang/stats','-n','1'],
                    capture_output=True,text=True,timeout=8)
                (a.output/(name+'_stats.txt')).write_text(q.stdout)
                match=re.search(r'iterations: (\d+)',q.stdout)
                assert match and int(match[1])>500, q.stdout
                summary[name]={'sdf_check':'pass','iterations':int(match[1]),'physics':'running'}
                poses=subprocess.run(['gz','topic','-e','-t','/world/gongchuang/pose/info','-n','1','--json-output'],
                    capture_output=True,text=True,timeout=8)
                live=json.JSONDecoder().raw_decode(poses.stdout.lstrip())[0]
                (a.output/(name+'_poses.json')).write_text(json.dumps(live,indent=2)+'\n')
                live_by_name={p['name']:p.get('position',{}) for p in live['pose']}
                expected={'field':[0,0], 'raw_material':cfg['stations']['raw']['center'],
                    'coarse':cfg['stations']['coarse']['center'],'temporary':cfg['stations']['temporary']['center']}
                expected.update({f'obstacle_{i+1}':pt for i,pt in enumerate(layouts.get(name,[]))})
                for entity,xy in expected.items():
                    point=live_by_name[entity]
                    assert abs(point.get('x',0)-xy[0])<1e-8 and abs(point.get('y',0)-xy[1])<1e-8, entity
                    assert abs(point.get('z',0))<.0001, entity
                summary[name]['live_entity_poses']='match_unified_geometry'
                scene=subprocess.run(['gz','service','-s','/world/gongchuang/scene/info',
                    '--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene','--timeout','5000','--req',''],
                    capture_output=True,text=True,timeout=10)
                (a.output/(name+'_scene.txt')).write_text(scene.stdout+scene.stderr)
                if name=='contact_test':
                    # Dynamic probe position is broadcast by SceneBroadcaster.
                    pose=subprocess.run(['gz','topic','-e','-t','/world/gongchuang/pose/info','-n','1'],
                        capture_output=True,text=True,timeout=8)
                    (a.output/'contact_probe_pose.txt').write_text(pose.stdout)
                    part=pose.stdout.split('name: "contact_probe"',1)[1].split('orientation',1)[0]
                    z=float(re.search(r'\bz: ([-+\d.eE]+)',part)[1])
                    assert abs(z-.05)<.002, f'Probe bottom is {z-.05}, expected 0'
                    summary[name]['settled_cylinder_center_z']=z
            finally:
                import signal
                os.killpg(proc.pid,signal.SIGINT)
                try: proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM); proc.wait(timeout=5)
        text=(a.output/(name+'_server.log')).read_text()
        if '[Err]' in text: raise RuntimeError('Gazebo server error: '+name+' (see log)')
        (a.output/'runtime_results.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(name,summary[name],flush=True)


if __name__=='__main__': main()
