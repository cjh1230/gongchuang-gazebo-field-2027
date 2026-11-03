"""PDF pages 4-8: physical turntable, six scoring rings, QR board and two-batch presets."""
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as E
from PIL import Image, ImageDraw, ImageFont
import yaml
from random_scenes import seeded_task


def t(parent,tagname,value=None,**attrs):
    e=E.SubElement(parent,tagname,{k:str(v) for k,v in attrs.items()})
    if value is not None: e.text=' '.join(map(str,value)) if isinstance(value,(list,tuple)) else str(value)
    return e


def inertia(link,mass,radius,height):
    i=t(link,'inertial'); t(i,'mass',mass); a=t(i,'inertia')
    for k,v in [('ixx',mass*(3*radius**2+height**2)/12),('iyy',mass*(3*radius**2+height**2)/12),('izz',mass*radius**2/2)]: t(a,k,v)


def write_stl(path,triangles):
    path.parent.mkdir(parents=True,exist_ok=True)
    text=['solid generated']
    for tri in triangles:
        a,b,c=tri
        u=[b[i]-a[i] for i in range(3)]; v=[c[i]-a[i] for i in range(3)]
        normal=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
        length=math.sqrt(sum(x*x for x in normal))
        text.extend(['facet normal '+' '.join(str(x/length) for x in normal),'outer loop'])
        text.extend('vertex '+' '.join(f'{v:.12g}' for v in pt) for pt in tri)
        text.extend(['endloop','endfacet'])
    text.append('endsolid generated'); path.write_text('\n'.join(text)+'\n')


def ring_mesh(path,radius,width):
    triangles=[]; n=192
    for i in range(n):
        a,b=2*math.pi*i/n,2*math.pi*(i+1)/n
        p=[(r*math.cos(theta),r*math.sin(theta),0) for r,theta in [(radius,a),(radius,b),(radius-width,b),(radius-width,a)]]
        triangles.extend([(p[0],p[1],p[2]),(p[0],p[2],p[3])])
    write_stl(path,triangles)


def frustum_mesh(path,r0,r1,height):
    triangles=[]; n=96
    for i in range(n):
        a,b=2*math.pi*i/n,2*math.pi*(i+1)/n
        p=[(r*math.cos(theta),r*math.sin(theta),z) for r,z,theta in [(r0,-height/2,a),(r0,-height/2,b),(r1,height/2,b),(r1,height/2,a)]]
        triangles.extend([(p[0],p[1],p[2]),(p[0],p[2],p[3]),((0,0,-height/2),p[1],p[0]),((0,0,height/2),p[3],p[2])])
    write_stl(path,triangles)


def appearance(v,color):
    m=t(v,'material'); t(m,'ambient',color); t(m,'diffuse',color); t(m,'specular',[.02,.02,.02,1])


def mesh_solid(link,name,pose,uri,color,collision=False):
    for kind in ['visual','collision'] if collision else ['visual']:
        v=t(link,kind,name=name+('_collision' if kind=='collision' else '')); t(v,'pose',pose)
        t(t(t(v,'geometry'),'mesh'),'uri',uri)
        if kind=='visual': appearance(v,color)


def painted_plane(link,name,pose,size,texture):
    v=t(link,'visual',name=name); t(v,'pose',pose)
    plane=t(t(v,'geometry'),'plane'); t(plane,'normal',[0,0,1]); t(plane,'size',size)
    m=t(v,'material'); t(m,'ambient',[1,1,1,1]); t(m,'diffuse',[1,1,1,1])
    metal=t(t(m,'pbr'),'metal'); t(metal,'albedo_map',texture); t(metal,'roughness',1); t(metal,'metalness',0)


def mission_data(root,c):
    mission=yaml.safe_load((root/'config/mission.yaml').read_text())
    code=mission['task_code']; provenance=mission['provenance']
    if mission.get('mode','manual')=='seeded':
        settings=yaml.safe_load((root/'config/random_generation.yaml').read_text())
        code=seeded_task(c.get('_generation_seed',settings['seed'])); provenance='synthetic_seeded_colors_and_task_not_official_draw'
    groups=code.split('+')
    if len(groups)!=4 or any(len(s)!=3 or not s.isdigit() for s in groups): raise ValueError('Task code requires four groups of 3 digits')
    colors1,slots1,colors2,slots2=[list(map(int,s)) for s in groups]
    if sorted(slots1)!=[1,2,3] or sorted(slots2)!=[1,2,3]: raise ValueError('Position groups must be permutations of 123')
    if len(set(colors1))!=3 or sorted(colors1)!=sorted(colors2) or any(i not in range(1,7) for i in colors1): raise ValueError('Two batches need the same 3 distinct colors')
    temp_slot=dict(zip(colors1,slots1))
    return {'task_code':code,'provenance':provenance,
        'batch1':{'colors':colors1,'coarse_slots':slots1,'temporary_slots':slots1},
        'batch2':{'colors':colors2,'coarse_slots':slots2,'temporary_slots':[temp_slot[i] for i in colors2]},
        'rules':{'one_item_per_grasp':True,'load_on_robot_before_cross_region':True,
            'batch2_stack_on_same_color':True,'return_to_drawn_start':True},
        'spawn_clearance':mission['spawn_clearance'],'stages':mission['stages'],
        'note':mission['note']}


def board_bounds(c):
    q=c['qr_board']; x,y,_=q['center']; angle=q['yaw']
    half_x=(q['thickness']*abs(math.cos(angle))+q['size'][0]*abs(math.sin(angle)))/2
    half_y=(q['thickness']*abs(math.sin(angle))+q['size'][0]*abs(math.cos(angle)))/2
    return [x-half_x,y-half_y,x+half_x,y+half_y]


def build_models(c,root,model,solid,save):
    raw=c['stations']['raw']; h,dt=raw['total_height'],raw['deck_thickness']
    s,m,base=model('raw_material_station'); m.find('static').text='false'
    solid(base,'pedestal',[0,0,(h-dt)/2,0,0,0],[raw['pedestal_diameter']/2,h-dt],[.74,.75,.77,1],'cylinder')
    inertia(base,1.,raw['pedestal_diameter']/2,h-dt)
    rotor=t(m,'link',name='turntable'); t(rotor,'pose',[0,0,h-dt/2,0,0,0])
    inertia(rotor,.5,raw['diameter']/2,dt)
    solid(rotor,'turntable',[0,0,0,0,0,0],[raw['diameter']/2,dt],c['field']['white_rgba'],'cylinder')
    for i in range(3):
        angle=-math.pi/2+i*2*math.pi/3
        solid(rotor,f'registration_{i}',[.1*math.cos(angle),.1*math.sin(angle),dt/2+.0002,0,0,0],[.009,.0001],[.2,.2,.2,1],'cylinder',False)
    fixed=t(m,'joint',name='fixture_to_world',type='fixed'); t(fixed,'parent','world'); t(fixed,'child','body')
    joint=t(m,'joint',name='turntable_joint',type='revolute'); t(joint,'parent','body'); t(joint,'child','turntable')
    ax=t(joint,'axis'); t(ax,'xyz',[0,0,1]); limit=t(ax,'limit'); t(limit,'lower',-1e16); t(limit,'upper',1e16); t(limit,'effort',10); t(limit,'velocity',2)
    settings=raw['rotation']
    if settings['enabled']:
        plugin=t(m,'plugin',filename='libgongchuang_turntable.so',name='gongchuang::TurntableSystem')
        for key in ['seconds_per_revolution','direction','seed','stops_per_revolution','dwell_seconds','initial_dwell_seconds']: t(plugin,key,settings[key])
    save(s,'raw_material_station')
    s,m,l=model('processing_station'); a,b=c['stations']['coarse']['size']; ph=c['stations']['coarse']['height']
    solid(l,'station_pad',[0,0,ph/2,0,0,0],[a,b,ph],c['field']['white_rgba'])
    diameter=c['materials']['bottom_diameter']
    if abs(diameter-c['targets']['material_diameter'])>1e-9: raise ValueError('Target rings must use the actual material maximum diameter')
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',190)
    for j,increment in enumerate(c['targets']['ring_diameter_increments']):
        diameter+=increment
        ring_mesh(root/f'models/processing_station/meshes/ring_{j+1}.stl',diameter/2,c['targets']['ring_line_width'])
        for slot,x in c['targets']['slot_local_x'].items():
            mesh_solid(l,f'slot_{slot}_ring_{j+1}',[x,0,ph+.00015,0,0,0],f'model://processing_station/meshes/ring_{j+1}.stl',[.025,.025,.025,1])
    for slot,x in c['targets']['slot_local_x'].items():
        img=Image.new('RGB',(256,256),'white'); d=ImageDraw.Draw(img)
        d.text((128,128),str(slot),font=font,anchor='mm',fill='black')
        dest=root/f'models/processing_station/materials/textures/number_{slot}.png'; dest.parent.mkdir(parents=True,exist_ok=True); img.save(dest)
        painted_plane(l,f'slot_{slot}_number',[x,0,ph+.0002,0,0,math.pi],[.015,.015],f'model://processing_station/materials/textures/number_{slot}.png')
    save(s,'processing_station')
    material=c['materials']; r0=material['bottom_diameter']/2; r1=material['top_diameter']/2; mh=material['height']; fh=material['flange_height']
    bodyh=mh-2*fh
    for color,props in material['colors'].items():
        name=f'material_{color}'; s,m,l=model(name); m.find('static').text='false'
        inertia(l,material['mass'],r0,mh)
        meshpath=root/f'models/{name}/meshes/taper.stl'; frustum_mesh(meshpath,r0,r1,bodyh)
        solid(l,'bottom_flange',[0,0,-mh/2+fh/2,0,0,0],[r0,fh],props['rgba'],'cylinder')
        mesh_solid(l,'taper',[0,0,0,0,0,0],f'model://{name}/meshes/taper.stl',props['rgba'],True)
        solid(l,'top_flange',[0,0,mh/2-fh/2,0,0,0],[r1,fh],props['rgba'],'cylinder')
        save(s,name)
    # Workspace-local build dependency, never globally installed by this generator.
    sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'.python_deps'))
    import qrcode
    payload=mission_data(root,c)['task_code']
    qr=qrcode.QRCode(border=4,box_size=10,error_correction=qrcode.constants.ERROR_CORRECT_M)
    qr.add_data(payload); qr.make(fit=True)
    qrimg=qr.make_image(fill_color='black',back_color='white').convert('RGB')
    size=c['qr_board']['size']; factor=10000
    board=Image.new('RGB',(round(size[0]*factor),round(size[1]*factor)),'white')
    side=round(c['qr_board']['code_size']*factor)
    qrimg=qrimg.resize((side,side),Image.Resampling.NEAREST)
    board.paste(qrimg,((board.width-side)//2,(board.height-side)//2))
    dest=root/'models/qr_board/materials/textures/task_board.png'; dest.parent.mkdir(parents=True,exist_ok=True); board.save(dest)
    s,m,l=model('qr_board')
    solid(l,'board',[0,0,0,0,0,0],[c['qr_board']['thickness'],*size],[1,1,1,1])
    painted_plane(l,'task_code',[-c['qr_board']['thickness']/2-.0001,0,0,math.pi/2,0,-math.pi/2],size,'model://qr_board/materials/textures/task_board.png')
    save(s,'qr_board')
    data=mission_data(root,c); data['qr_matrix']=qr.get_matrix()
    (root/'config/generated_mission.json').write_text(json.dumps(data,indent=2)+'\n')


def add_materials(w,c,root,stage,include):
    data=mission_data(root,c); state=data['stages'].get(stage,'batch1_raw')
    mh=c['materials']['height']; epsilon=data['spawn_clearance']; count=0
    def put(color,batch,index,location,slot=None,layer=0):
        nonlocal count
        if location=='raw':
            a=-math.pi/2+index*2*math.pi/3; rr=c['materials']['raw_orbit_diameter']/2
            x,y=c['stations']['raw']['center']; x+=rr*math.cos(a); y+=rr*math.sin(a)
            z=c['stations']['raw']['total_height']+mh/2+epsilon
        else:
            station=c['stations'][location]; lx=c['targets']['slot_local_x'][slot]
            x=station['center'][0]+lx*math.cos(station['yaw']); y=station['center'][1]+lx*math.sin(station['yaw'])
            z=station['height']+mh/2+layer*mh+epsilon
        include(w,f'material_{color}',f'batch{batch}_item{index+1}_color{color}',[x,y,z,0,0,0]); count+=1
    b1,b2=data['batch1'],data['batch2']
    if state=='batch1_raw':
        for i,color in enumerate(b1['colors']): put(color,1,i,'raw')
    if state=='batch1_coarse':
        for i,color in enumerate(b1['colors']): put(color,1,i,'coarse',b1['coarse_slots'][i])
    if state in ['batch1_stored_batch2_raw','batch1_stored_batch2_coarse','both_batches_stacked_by_color']:
        for i,color in enumerate(b1['colors']): put(color,1,i,'temporary',b1['temporary_slots'][i])
        for i,color in enumerate(b2['colors']):
            if state=='batch1_stored_batch2_raw': put(color,2,i,'raw')
            elif state=='batch1_stored_batch2_coarse': put(color,2,i,'coarse',b2['coarse_slots'][i])
            else: put(color,2,i,'temporary',b2['temporary_slots'][i],1)
    return count
