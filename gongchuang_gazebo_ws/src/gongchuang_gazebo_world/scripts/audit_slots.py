#!/usr/bin/env python3
"""Reproduce 1000-seed statistics and a coordinate/reference diagram from source config."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import yaml
from random_scenes import draw_ids, validate_slots, Validator, select_scene

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True)
    c=yaml.safe_load((ROOT/'config/field_geometry.yaml').read_text())
    s=yaml.safe_load((ROOT/'config/random_generation.yaml').read_text()); v=Validator(c,s)
    positions=validate_slots(c,s); freq=Counter(); combinations=Counter(); statuses=Counter(); rows=[]
    for seed in range(1000):
        pts,info,_,result=select_scene(c,s,seed,validator=v)
        ids=info['selected_slots']; assert len(ids)==2 and len(set(ids))==2
        assert ids==draw_ids(s,seed)
        freq.update(ids); combinations.update([','.join(sorted(ids,key=lambda x:int(x[1:])))])
        statuses.update([result['status']]); rows.append({'seed':seed,'slots':ids,'status':result['status']})
    expectation=2000/13
    stats={'seed_range':[0,999],'draws':1000,'count_per_draw':2,'replacement':False,
        'slot_frequency':{f'R{i}':freq[f'R{i}'] for i in range(1,14)},'expected_per_slot':expectation,
        'unique_combinations':len(combinations),'possible_combinations':78,
        'combination_frequency':dict(sorted(combinations.items())),
        'pearson_frequency_diagnostic':sum((n-expectation)**2/expectation for n in freq.values()),
        'note':'Diagnostic only; draws have dependent within-draw slot indicators. Different seeds can repeat combinations.',
        'task_statuses':dict(statuses),'navigation_task':s['navigation_task'],'duplicate_draws':0}
    (a.output/'distribution_1000.json').write_text(json.dumps(stats,indent=2)+'\n')
    (a.output/'draws_1000.json').write_text(json.dumps(rows,indent=2)+'\n')
    (a.output/'candidate_coordinates.json').write_text(json.dumps(positions,indent=2)+'\n')
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="840" height="980" viewBox="0 0 840 980">',
        '<rect width="840" height="980" fill="#fafbfc"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#18394d} .id{font-size:22px;font-weight:bold}</style>',
        '<text x="70" y="38" font-size="25">R1–R13 reference slots · Gazebo map frame</text>',
        '<text x="70" y="66" font-size="16">Uncalibrated geometry-aligned estimates · not official coordinates</text>',
        '<g transform="translate(70 875) scale(300 -300)">',
        '<rect width="2.4" height="2.4" fill="#aaaab0" stroke="#555" stroke-width=".007"/>']
    for x,y in c['field']['yellow_lower_left']:
        svg.append(f'<rect x="{x}" y="{y}" width=".45" height=".45" fill="#ffffcc"/>')
    for st in c['field']['starts']:
        x,y=st['lower_left']; svg.append(f'<rect x="{x}" y="{y}" width=".3" height=".3" fill="#1125ff"/>')
    svg.extend(['<rect x=".91" y="0" width=".58" height=".15" fill="white"/>',
        '<rect x="0" y=".91" width=".15" height=".58" fill="white"/>',
        '<circle cx="1.2" cy="2.47" r=".15" fill="white" stroke="#555" stroke-width=".006"/>'])
    for e in positions:
        x,y=e['xy_m']; svg.append(f'<circle cx="{x}" cy="{y}" r=".025" fill="#172e3f"/>')
    svg.append('</g>')
    for e in positions:
        x,y=e['xy_m']; px,py=70+300*x,875-300*y
        svg.append(f'<text class="id" x="{px}" y="{py-17}" text-anchor="middle">{e["slot_id"]}</text>')
    svg.extend(['<text x="70" y="920" font-size="17">Origin (0,0) lower left · X → right · Y ↑ up · units: m</text>',
        '<text x="70" y="949" font-size="17">Cross: north R10 · west R11 · center R13 · east R9 · south R12</text>','</svg>'])
    (a.output/'candidate_slots.svg').write_text('\n'.join(svg),encoding='utf-8')
    print(json.dumps(stats),flush=True)


if __name__=='__main__': main()
