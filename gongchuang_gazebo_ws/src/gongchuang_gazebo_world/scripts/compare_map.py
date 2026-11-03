#!/usr/bin/env python3
"""Compare an unmodified Gazebo top capture with exported rule maps and check reprojection."""
import argparse
import json
import math
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
if not (ROOT/'maps').is_dir():
    from ament_index_python.packages import get_package_share_directory
    ROOT=Path(get_package_share_directory('gongchuang_gazebo_world'))


def main():
    p=argparse.ArgumentParser(); p.add_argument('--screenshots',type=Path,required=True)
    p.add_argument('--bundle',type=Path,default=ROOT)
    p.add_argument('--layout',default='competition')
    a=p.parse_args(); shot=Image.open(a.screenshots/'top.png').convert('RGB')
    w,h=shot.size; focal=w/(2*math.tan(1.05/2)); scale=focal/3.55
    def uv(x,y): return round(w/2+(x-1.2)*scale),round(h/2-(y-1.3)*scale)
    samples={}
    for name,x,y,kind in [('yellow_sw',.775,.775,'yellow'),('yellow_ne',1.625,1.625,'yellow'),
            ('start_1',2.25,2.25,'blue'),('start_2',2.25,.15,'blue'),('gray',1.2,1.5,'gray')]:
        point=uv(x,y); r,g,b=shot.getpixel(point)
        match=(r-b>12 and g-b>12 and r>180) if kind=='yellow' else b>r*1.6 if kind=='blue' else max(r,g,b)-min(r,g,b)<15
        assert match, (name,point,(r,g,b))
        samples[name]={'world_xy':[x,y],'image_uv':point,'rgb':[r,g,b],'pass':True}
    # Measure the gap between the yellow panels in the actual image, then project back to meters.
    yy=uv(0,.7)[1]
    yellow_x=[x for x in range(w) if (lambda rgb:rgb[0]-rgb[2]>12 and rgb[1]-rgb[2]>12 and rgb[0]>180)(shot.getpixel((x,yy)))]
    west=[x for x in yellow_x if x<w/2]; east=[x for x in yellow_x if x>w/2]
    measured=(min(east)-max(west)-1)/scale
    assert abs(measured-.4)<.008, measured
    report={'camera':'audit_top','projection':'pinhole from SDF pose/FOV', 'meters_per_pixel':1/scale,
        'central_lane_render_measurement_m':measured,'tolerance_m':.008,'samples':samples,
        'note':'SDF tests establish exact dimensions; raster reprojection is an independent rendering check.'}
    (a.screenshots/'render_map_alignment.json').write_text(json.dumps(report,indent=2)+'\n')
    fontpath='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font=ImageFont.truetype(fontpath,22); small=ImageFont.truetype(fontpath,17)
    canvas=Image.new('RGB',(1800,790),'#f5f6f8'); d=ImageDraw.Draw(canvas)
    d.text((30,22),'2027 Gongchuang | Gazebo geometry and planning layers',font=font,fill='#16202b')
    size=540; y0=110
    x0,y1=uv(0,2.4); x1,y2=uv(2.4,0)
    crop=shot.crop((x0,y1,x1,y2)).resize((size,size))
    palette={1:(190,190,195),2:(255,255,204),3:(30,60,240),4:(30,60,240),5:(240,240,240),6:(160,110,180),7:(0,0,0),8:(60,85,100)}
    semantic=Image.open(a.bundle/'maps'/(a.layout+'_semantic.pgm'))
    color=Image.new('RGB',semantic.size); color.putdata([palette[v] for v in semantic.getdata()])
    occ=Image.open(a.bundle/'maps'/(a.layout+'_transit.pgm')).convert('RGB')
    for i,(title,img) in enumerate([('Actual Gazebo top capture',crop),('Semantic rules (nominal)',color),('Transit occupancy (blue closed)',occ)]):
        x=30+i*590; d.text((x,72),title,font=font,fill='#253246')
        canvas.paste(img.resize((size,size),Image.Resampling.NEAREST),(x,y0))
        d.rectangle((x,y0,x+size,y0+size),outline='#516070',width=1)
        d.text((x,y0+size+12),'(0,0) lower-left | +X right | +Y up',font=small,fill='#253246')
    d.text((30,710),'Gray: legal transit | Yellow: forbidden | Blue: selected start/return only | Purple: raw placement unknown',font=small,fill='#253246')
    d.text((30,741),'480 x 480 cells | 5 mm/cell | world = map | fine/finished stations: not located in official preliminary figure',font=small,fill='#253246')
    canvas.save(a.screenshots/'map_comparison.png')
    print(json.dumps({'alignment':'pass','lane_render_m':measured,'image':'map_comparison.png'}))


if __name__=='__main__': main()
