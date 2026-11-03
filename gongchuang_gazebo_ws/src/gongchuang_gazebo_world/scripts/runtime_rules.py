#!/usr/bin/env python3
"""Measure real joint motion, dwell, material transport and stable two-batch stacks."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import statistics
import time
import xml.etree.ElementTree as E

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True)
    os.environ['GZ_PARTITION']='gongchuang_rules_validation'
    os.environ['GZ_SIM_RESOURCE_PATH']=str(ROOT/'models')
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(ROOT.parents[1]/'install/gongchuang_gazebo_world/lib')
    from gz.transport13 import Node
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.msgs10.pose_v_pb2 import Pose_V
    node=Node(); status=[]; poses=[]
    def on_status(msg): status.append(json.loads(msg.data))
    def on_pose(msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        if poses and stamp-poses[-1]['time']<.1: return
        points={p.name:[p.position.x,p.position.y,p.position.z] for p in msg.pose if p.name.startswith('batch')}
        poses.append({'time':stamp,'materials':points})
    node.subscribe(StringMsg,'/gongchuang/turntable/status',on_status)
    node.subscribe(Pose_V,'/world/gongchuang/pose/info',on_pose)
    results={}
    colors=json.loads((ROOT/'config/generated_mission.json').read_text())['batch1']['colors']
    for case,direction,iterations in [('forward',1,12000),('reverse',-1,12000),('stacking_demo',0,6000),('batch2_ready',0,6000),('batch2_coarse',0,6000),('batch1_coarse',0,4000)]:
        status.clear(); poses.clear()
        if direction:
            tree=E.parse(ROOT/'worlds/competition.sdf'); w=tree.getroot().find('world')
            inc=w.find("include[name='raw_material']"); pose_text=inc.findtext('pose'); w.remove(inc)
            raw=E.parse(ROOT/'models/raw_material_station/model.sdf').getroot().find('model')
            raw.set('name','raw_material'); E.SubElement(raw,'pose').text=pose_text
            raw.find("plugin[@name='gongchuang::TurntableSystem']/direction").text=str(direction)
            w.append(raw); file=a.output/(case+'.sdf'); tree.write(file)
        else: file=ROOT/'worlds'/(case+'.sdf')
        proc=subprocess.run(['gz','sim','-s','-r','-v','3','--iterations',str(iterations),str(file)],
            capture_output=True,text=True,timeout=50)
        log=proc.stdout+proc.stderr; (a.output/(case+'.log')).write_text(log)
        assert proc.returncode==0 and '[Err]' not in log, case+' server error'
        time.sleep(.3)
        (a.output/(case+'_samples.json')).write_text(json.dumps({'joint':status,'poses':poses},indent=2)+'\n')
        assert status and poses and max(s['sim_time'] for s in status)>iterations/1000-.5,case+' missing telemetry'
        result={'server':'pass','samples':len(status)}
        if direction:
            moving=[abs(s['velocity']) for s in status if s['state']=='rotating' and abs(s['command'])>.7]
            expected=2*math.pi/8
            assert moving and abs(statistics.median(moving)-expected)<.002,(case,moving[:10])
            dwells=[s for s in status if s['state']=='dwell' and s['sim_time']>1]
            assert max(abs(s['velocity']) for s in dwells)<.002,case+' not stopped'
            stop_angles=sorted(set(round(s['target'],4) for s in dwells))
            assert len(stop_angles)>=3,stop_angles
            for s in dwells: assert abs(s['angle']-s['target'])<.001, s
            for target in stop_angles:
                group=[s['sim_time'] for s in dwells if abs(s['target']-target)<.0001]
                if min(group)>2 and max(group)<11.9: assert max(group)-min(group)>1.8,group
            result.update({'seconds_per_active_revolution':2*math.pi/statistics.median(moving),
                'direction':direction,'distinct_dwell_angles':stop_angles,'max_dwell_velocity':max(abs(s['velocity']) for s in dwells)})
            last=next(p['materials'] for p in reversed(poses) if len(p['materials'])==3)
            for point in last.values():
                assert abs(math.hypot(point[0]-1.2,point[1]-2.47)-.1)<.008,point
                assert abs(point[2]-.12)<.003,point
            result['free_materials_carried_by_friction']='pass'
        else:
            last=next(p['materials'] for p in reversed(poses) if len(p['materials'])>0)
            expected_count=3 if case=='batch1_coarse' else 6
            assert len(last)==expected_count, (case,last)
            if case=='stacking_demo':
                for color in colors:
                    pair=[point for name,point in last.items() if name.endswith(f'color{color}')]
                    assert len(pair)==2
                    pair.sort(key=lambda p:p[2])
                    assert abs(pair[0][2]-.04)<.002 and abs(pair[1][2]-.1)<.002,pair
                    assert math.dist(pair[0][:2],pair[1][:2])<.003,pair
                    recent=[p['materials'] for p in poses if p['time']>4.5]
                    for name,point in last.items():
                        assert max(math.dist(point,s[name]) for s in recent if name in s)<.002,(name,'stack unstable')
                result['three_same_color_two_level_physical_stacks']='pass'
            result['material_count']=len(last)
        results[case]=result
        (a.output/'rules_runtime_results.json').write_text(json.dumps(results,indent=2)+'\n')
        print(case,result,flush=True)
    node.unsubscribe('/gongchuang/turntable/status'); node.unsubscribe('/world/gongchuang/pose/info')


if __name__=='__main__': main()
