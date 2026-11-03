#!/usr/bin/env python3
"""Generate SDF and planning layers from field_geometry.yaml. No online assets."""
import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path
import xml.etree.ElementTree as E
import yaml
import scene_details as details
from random_scenes import select_scene, preset_scene, Validator, validate_slots, MODES

ROOT = Path(__file__).resolve().parents[1]
if not (ROOT/'config').is_dir():
    from ament_index_python.packages import get_package_share_directory
    ROOT=Path(get_package_share_directory('gongchuang_gazebo_world'))


def tag(parent, tagname, value=None, **attrs):
    e = E.SubElement(parent, tagname, {k: str(v) for k, v in attrs.items()})
    if value is not None:
        e.text = ' '.join(map(str, value)) if isinstance(value, (list, tuple)) else str(value)
    return e


def save_xml(root, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    E.indent(root, space='  ')
    E.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def model(name):
    s = E.Element('sdf', version='1.9')
    m = tag(s, 'model', name=name)
    tag(m, 'static', 'true')
    return s, m, tag(m, 'link', name='body')


def geometry(parent, shape, dims):
    g = tag(parent, 'geometry')
    g = tag(g, shape)
    if shape == 'box':
        tag(g, 'size', dims)
    else:
        tag(g, 'radius', dims[0])
        tag(g, 'length', dims[1])


def solid(link, name, pose, dims, color, shape='box', collision=True):
    v = tag(link, 'visual', name=name)
    tag(v, 'pose', pose)
    geometry(v, shape, dims)
    mat = tag(v, 'material')
    tag(mat, 'ambient', color)
    tag(mat, 'diffuse', color)
    tag(mat, 'specular', [0.03, 0.03, 0.03, 1])
    if collision:
        c = tag(link, 'collision', name=name + '_collision')
        tag(c, 'pose', pose)
        geometry(c, shape, dims)
        surf = tag(c, 'surface')
        ode = tag(tag(surf, 'friction'), 'ode')
        tag(ode, 'mu', 0.9)
        tag(ode, 'mu2', 0.9)


def model_save(sdf, name):
    folder = ROOT / 'models' / name
    save_xml(sdf, folder / 'model.sdf')
    c = E.Element('model')
    tag(c, 'name', name)
    tag(c, 'version', '1.0')
    tag(c, 'sdf', 'model.sdf', version='1.9')
    tag(c, 'description', 'Generated from config/field_geometry.yaml; see provenance.md.')
    save_xml(c, folder / 'model.config')


def box_bounds(station):
    x, y = station['center']
    a, b = station['size']
    if abs(station['yaw']) > 1:
        a, b = b, a
    return [x-a/2, y-b/2, x+a/2, y+b/2]


def inside(x, y, b):
    return b[0] <= x < b[2] and b[1] <= y < b[3]


def build_models(c):
    f, stations = c['field'], c['stations']
    sx, sy = f['size']
    t = f['slab_thickness']
    s, m, l = model('competition_field')
    solid(l, 'ground', [sx/2, sy/2, -t/2, 0, 0, 0], [sx, sy, t], f['gray_rgba'])
    # Visual sheets only: no collision and no step at colored boundaries.
    for i, (x, y) in enumerate(f['yellow_lower_left']):
        a, b = f['yellow_size']
        solid(l, 'yellow_' + str(i+1), [x+a/2, y+b/2, .00015, 0, 0, 0],
              [a, b, .0001], f['yellow_rgba'], collision=False)
    for start in f['starts']:
        x, y = start['lower_left']
        a, b = f['start_size']
        solid(l, start['id'], [x+a/2, y+b/2, .00015, 0, 0, 0],
              [a, b, .0001], f['blue_rgba'], collision=False)
    model_save(s, 'competition_field')
    s, m, l = model('cylinder_obstacle')
    ob = c['obstacle']
    solid(l, 'cylinder', [0, 0, ob['height']/2, 0, 0, 0],
          [ob['diameter']/2, ob['height']], [.015, .015, .015, 1], 'cylinder')
    model_save(s, 'cylinder_obstacle')
    details.build_models(c,ROOT,model,solid,model_save)


def include(w, uri, name, pose):
    inc = tag(w, 'include')
    tag(inc, 'uri', 'model://' + uri)
    tag(inc, 'name', name)
    tag(inc, 'pose', pose)


def build_slot_debug(settings):
    """Visuals only; labels are part of Gazebo rendering, never screenshot overlays."""
    from PIL import Image, ImageDraw, ImageFont
    sdf, mo, li = model('slot_debug')
    folder=ROOT/'models/slot_debug'; folder.mkdir(parents=True,exist_ok=True)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',72)
    for sid,e in settings['candidate_slots']['slots'].items():
        x,y=e['xy_m']; image=Image.new('RGB',(256,128),(238,245,247))
        draw=ImageDraw.Draw(image); draw.text((128,64),sid,font=font,fill=(15,65,95),anchor='mm')
        image.save(folder/(sid+'.png'))
        details.painted_plane(li,sid+'_label',[x,y+.085,.002,0,0,0],[.14,.07],'model://slot_debug/'+sid+'.png')
        details.ring_mesh(folder/(sid+'.stl'),.035,.004)
        details.mesh_solid(li,sid+'_marker',[x,y,.002,0,0,0],'model://slot_debug/'+sid+'.stl',[.05,.7,.9,1])
    model_save(sdf,'slot_debug')


def world(c, name, points, audit=False, probe=False, debug=False):
    s = E.Element('sdf', version='1.9')
    w = tag(s, 'world', name='gongchuang')
    p = tag(w, 'physics', name='physics', type='ignored')
    tag(p, 'max_step_size', .001)
    tag(p, 'real_time_factor', 1)
    tag(w, 'gravity', [0, 0, -9.81])
    for filename, cls in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                          ('scene-broadcaster', 'SceneBroadcaster')]:
        tag(w, 'plugin', filename='gz-sim-'+filename+'-system', name='gz::sim::systems::'+cls)
    scene = tag(w, 'scene')
    tag(scene, 'ambient', [.55, .55, .55, 1])
    tag(scene, 'background', [.19, .21, .24, 1])
    tag(scene, 'shadows', 'true')
    tag(scene, 'grid', 'false')
    light = tag(w, 'light', name='sun', type='directional')
    tag(light, 'pose', [0, 0, 5, 0, 0, 0])
    tag(light, 'diffuse', [.9, .9, .9, 1])
    tag(light, 'specular', [.1, .1, .1, 1])
    tag(light, 'direction', [-.3, .2, -1])
    tag(light, 'cast_shadows', 'true')
    att = tag(light, 'attenuation')
    tag(att, 'range', 100)
    tag(att, 'constant', .9)
    tag(att, 'linear', .01)
    tag(att, 'quadratic', .001)
    include(w, 'competition_field', 'field', [0, 0, 0, 0, 0, 0])
    raw = c['stations']['raw']
    include(w, 'raw_material_station', 'raw_material', [*raw['center'], 0, 0, 0, 0])
    for station in ['coarse', 'temporary']:
        cfg = c['stations'][station]
        include(w, 'processing_station', station, [*cfg['center'], 0, 0, 0, cfg['yaw']])
    board=c['qr_board']
    include(w,'qr_board','task_board',[*board['center'],0,0,board['yaw']])
    if not probe:
        stage='stacking_demo' if name in ['audit_stacking','audit_random_stacking'] else name.removeprefix('random_')
        details.add_materials(w,c,ROOT,stage,include)
    for i, (x, y) in enumerate(points):
        include(w, 'cylinder_obstacle', 'obstacle_'+str(i+1), [x, y, 0, 0, 0, 0])
    if debug:
        include(w,'slot_debug','candidate_slot_debug',[0,0,0,0,0,0])
    # GUI uses only local plugins and defaults, no Fuel lookup.
    gui = tag(w, 'gui', fullscreen='false')
    for fn, nm in [('MinimalScene', '3D View'), ('GzSceneManager', 'Scene Manager'), ('CameraTracking', 'Camera Tracking'),
                   ('InteractiveViewControl', 'View control'), ('WorldControl', 'World control'),
                   ('WorldStats', 'World statistics'), ('EntityTree', 'Entity tree'), ('Screenshot','Screenshot')]:
        pl = tag(gui, 'plugin', filename=fn, name=nm)
        ui = tag(pl, 'gz-gui')
        tag(ui,'title',nm)
        tag(ui,'property','false',type='bool',key='showTitleBar')
        tag(ui,'property','docked' if fn in ['MinimalScene','EntityTree'] else 'floating',type='string',key='state')
        if fn in ['GzSceneManager','InteractiveViewControl','CameraTracking']:
            tag(ui,'property',5,type='double',key='width')
            tag(ui,'property',5,type='double',key='height')
        if fn in ['WorldControl','WorldStats','Screenshot']:
            tag(ui,'property',72 if fn!='WorldStats' else 110,type='double',key='height')
            tag(ui,'property',50 if fn=='Screenshot' else 290,type='double',key='width')
            tag(ui,'property',1,type='double',key='z')
            anchors=tag(ui,'anchors',target='3D View')
            tag(anchors,'line',own='right' if fn=='WorldStats' else 'left',target='right' if fn=='WorldStats' else 'left')
            tag(anchors,'line',own='top' if fn=='Screenshot' else 'bottom',target='top' if fn=='Screenshot' else 'bottom')
        if fn == 'MinimalScene':
            tag(pl, 'engine', 'ogre2')
            tag(pl, 'scene', 'scene')
            tag(pl, 'ambient_light', [.55, .55, .55])
            tag(pl, 'background_color', [.19, .21, .24])
            tag(pl, 'camera_pose', [1.2,1.3,3.5,0,math.pi/2,math.pi/2] if debug else [2.9, -1.1, 2.8, 0, .775, 2.207])
            tag(pl, 'horizontal_fov', 55)
            tag(pl, 'anti_aliasing', 4)
            tag(pl, 'camera_clip', None)
            tag(pl[-1], 'near', .01)
            tag(pl[-1], 'far', 100)
        if fn=='WorldControl':
            tag(pl,'play_pause','true'); tag(pl,'step','true'); tag(pl,'use_event','true')
        if fn=='WorldStats':
            for metric in ['sim_time','real_time','real_time_factor','iterations']: tag(pl,metric,'true')
    if audit:
        pl = tag(w, 'plugin', filename='gz-sim-sensors-system', name='gz::sim::systems::Sensors')
        tag(pl, 'render_engine', 'ogre2')
        cameras = {
            'top': [1.2, 1.30, 3.55, 0, math.pi/2, math.pi/2],
            'isometric': [3.4, -1.7, 3.3, 0, .73, 2.22],
            'cross': [1.2, 1.2, 1.6, 0, math.pi/2, math.pi/2],
            'stations': [1.2, 2.4, .9, 0, math.pi/2, math.pi/2],
            'coarse': [1.2, .15, .75, 0, math.pi/2, math.pi/2],
            'temporary': [.15, 1.2, .75, 0, math.pi/2, math.pi/2],
            'obstacles': [1.82, .05, 1.45, 0, .9, 2.02],
            'qr_board': [1.9, 1.2, .20, 0, 0, 0],
            'raw_close': [1.5, 1.98, .42, 0, .5, 2.12],
            'stack_close': [.65, .62, .40, 0, .36, 2.36],
        }
        for name_cam, pose in cameras.items():
            mo = tag(w, 'model', name='audit_'+name_cam)
            tag(mo, 'static', 'true')
            tag(mo, 'pose', pose)
            li = tag(mo, 'link', name='camera_link')
            sn = tag(li, 'sensor', name='camera', type='camera')
            tag(sn, 'always_on', 'true')
            tag(sn, 'update_rate', 2)
            tag(sn, 'topic', '/audit/'+name_cam)
            ca = tag(sn, 'camera')
            tag(ca, 'horizontal_fov', 1.05)
            im = tag(ca, 'image')
            tag(im, 'width', 1400)
            tag(im, 'height', 1200)
            tag(im, 'format', 'R8G8B8')
            clip = tag(ca, 'clip')
            tag(clip, 'near', .01)
            tag(clip, 'far', 20)
    if probe:
        mo = tag(w, 'model', name='contact_probe')
        tag(mo, 'pose', [1.2, 1.2, .3, 0, 0, 0])
        li = tag(mo, 'link', name='body')
        inert = tag(li, 'inertial')
        tag(inert, 'mass', .1)
        inertia = tag(inert, 'inertia')
        for key, value in [('ixx', .0001), ('iyy', .0001), ('izz', .00003)]:
            tag(inertia, key, value)
        solid(li, 'probe', [0, 0, 0, 0, 0, 0], [.025, .1], [.1, .1, .1, 1], 'cylinder')
    return s


def write_pgm(path, pixels, n):
    path.write_bytes(f'P5\n{n} {n}\n255\n'.encode() + bytes(pixels))


def build_maps(c, layouts):
    f, st, res = c['field'], c['stations'], c['map']['resolution']
    n = round(f['size'][0]/res)
    bounds_y = [[x, y, x+f['yellow_size'][0], y+f['yellow_size'][1]]
                for x, y in f['yellow_lower_left']]
    bounds_s = [[*s['lower_left'], s['lower_left'][0]+.3, s['lower_left'][1]+.3] for s in f['starts']]
    bounds_station = [box_bounds(st['coarse']), box_bounds(st['temporary'])]
    output = ROOT / 'maps'
    output.mkdir(exist_ok=True)
    for layout, obstacles in layouts.items():
        # Semantic IDs: 1 gray, 2 yellow, 3 start1, 4 start2, 5 station, 6 raw uncertain, 7 cylinder, 8 QR board.
        semantic, physical = [], []
        modes = {k: [] for k in ['transit', 'start_1', 'start_2']}
        for row in range(n):
            y = (n-row-.5)*res
            for col in range(n):
                x = (col+.5)*res
                kind = 1
                if any(inside(x,y,b) for b in bounds_y): kind = 2
                for i,b in enumerate(bounds_s):
                    if inside(x,y,b): kind = 3+i
                if any(inside(x,y,b) for b in bounds_station): kind = 5
                if inside(x,y,st['raw']['exclusion_bounds']): kind = 6
                board=details.board_bounds(c)
                board_hit=x+res/2>board[0] and x-res/2<board[2] and y+res/2>board[1] and y-res/2<board[3]
                if board_hit: kind=8
                hit = any(math.hypot(x-a,y-b) <= c['obstacle']['diameter']/2 for a,b in obstacles)
                raw_hit = math.hypot(x-st['raw']['center'][0], y-st['raw']['center'][1]) <= st['raw']['diameter']/2
                # Obstacle cells conservatively include any intersected cell, not only centers.
                def circle_cell(a,b,r):
                    dx,dy = max(abs(x-a)-res/2,0), max(abs(y-b)-res/2,0)
                    return dx*dx+dy*dy <= r*r
                hit = any(circle_cell(a,b,c['obstacle']['diameter']/2) for a,b in obstacles)
                raw_hit = circle_cell(*st['raw']['center'], st['raw']['diameter']/2)
                physical.append(0 if hit or raw_hit or kind in [5,8] else 254)
                if hit: kind = 7
                semantic.append(kind)
                for mode, pix in modes.items():
                    free = kind==1 or (mode=='start_1' and kind==3) or (mode=='start_2' and kind==4)
                    pix.append(254 if free else 205 if kind==6 else 0)
        for mode,pixels in {**modes, 'physical_nominal':physical}.items():
            stem = f'{layout}_{mode}'
            write_pgm(output/(stem+'.pgm'), pixels, n)
            (output/(stem+'.yaml')).write_text(yaml.safe_dump({
                'image':stem+'.pgm','mode':'trinary','resolution':res,'origin':c['map']['origin'],
                'negate':0,'occupied_thresh':.65,'free_thresh':.196},sort_keys=False),encoding='utf-8')
        write_pgm(output/(layout+'_semantic.pgm'), semantic,n)
    metadata = {'schema_version':1,'frame':'map','map_to_world':c['frame']['map_to_world'],
                'cell_center':'x=(col+0.5)*resolution; y=(height-row-0.5)*resolution',
                'semantic_ids':{'gray':1,'yellow':2,'start_1':3,'start_2':4,'station':5,'raw_uncertain':6,'cylinder':7,'qr_board':8},
                'unknown_pgm':205,'free_pgm':254,'blocked_pgm':0,
                'start_policy':c['map']['start_policy'],'observation_policy':c['map']['observation_policy'],
                'fine':st['fine'],'finished':st['finished'],
                'warning':'Rule occupied means forbidden, not necessarily solid. Do not replace local live observations with physical_nominal.'}
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')


def main():
    global ROOT
    parser=argparse.ArgumentParser()
    parser.add_argument('--seed',type=int)
    parser.add_argument('--obstacle-count',type=int)
    parser.add_argument('--preset',help='Named obstacle preset from config/random_generation.yaml')
    parser.add_argument('--mode',choices=MODES)
    parser.add_argument('--slots',nargs='*',help='Exact R IDs, requires --mode manual_slots')
    parser.add_argument('--debug-slots',action='store_true')
    parser.add_argument('--output',type=Path,help='Isolated generated bundle; source files remain unchanged')
    args=parser.parse_args()
    if args.preset and (args.mode or args.slots is not None): parser.error('--preset cannot be combined with --mode/--slots')
    source=ROOT
    if args.output:
        target=args.output.expanduser().resolve()
        marker=target/'.gongchuang_generated'
        if target.exists() and not marker.exists(): raise ValueError('Output exists without generated-bundle marker')
        (target/'config').mkdir(parents=True,exist_ok=True); marker.write_text('Managed generated scene bundle\n')
        for name in ['field_geometry.yaml','mission.yaml','obstacle_layouts.yaml','random_generation.yaml']:
            shutil.copy2(source/'config'/name,target/'config'/name)
        ROOT=target
    c = yaml.safe_load((ROOT/'config/field_geometry.yaml').read_text(encoding='utf-8'))
    layout_ids = yaml.safe_load((ROOT/'config/obstacle_layouts.yaml').read_text(encoding='utf-8'))['layouts']
    settings=yaml.safe_load((ROOT/'config/random_generation.yaml').read_text())
    seed=args.seed if args.seed is not None else settings['seed']
    validate_slots(c,settings)
    slots=settings['candidate_slots']['slots']
    layouts={name:[list(slots[s]['xy_m']) for s in ids] for name,ids in layout_ids.items()}
    count=args.obstacle_count
    validator=Validator(c,settings)
    if args.preset:
        if args.preset not in settings['presets']: raise ValueError('Unknown obstacle preset')
        random_points,randomization,certificate,assessment=preset_scene(c,settings,seed,args.preset,validator)
        if args.obstacle_count is not None and args.obstacle_count!=len(random_points):
            raise ValueError('Preset has a fixed obstacle count; omit --obstacle-count or use its exact count')
    else:
        random_points,randomization,certificate,assessment=select_scene(c,settings,seed,count,args.mode,args.slots,validator)
    count=len(random_points)
    c['_generation_seed']=seed; c['stations']['raw']['rotation']['seed']=seed
    layouts['random_seeded']=random_points
    (ROOT/'maps').mkdir(exist_ok=True)
    (ROOT/'config/generated_randomization.json').write_text(json.dumps(randomization,indent=2)+'\n')
    (ROOT/'maps/random_seeded_certificate.json').write_text(json.dumps(certificate,indent=2)+'\n')
    (ROOT/'maps/random_seeded_reachability.json').write_text(json.dumps(assessment,indent=2)+'\n')
    presets={}
    for name in settings['presets']:
        pts,info,cert,result=preset_scene(c,settings,seed,name,validator)
        layouts[name]=pts; presets[name]=info
        (ROOT/'maps'/(name+'_reachability.json')).write_text(json.dumps(result,indent=2)+'\n')
    (ROOT/'config/generated_presets.json').write_text(json.dumps(presets,indent=2)+'\n')
    if any(c['stations']['coarse'][k]!=c['stations']['temporary'][k] for k in ['size','height']):
        raise ValueError('Shared processing_station requires equal coarse/temporary size and height')
    build_models(c)
    build_slot_debug(settings)
    for name,points in layouts.items():
        save_xml(world(c,name,points,debug=args.debug_slots and name=='random_seeded'),ROOT/'worlds'/(name+'.sdf'))
    for stage in ['batch1_coarse','batch2_ready','batch2_coarse','stacking_demo']:
        save_xml(world(c,stage,layouts['competition']),ROOT/'worlds'/(stage+'.sdf'))
        save_xml(world(c,'random_'+stage,random_points,debug=args.debug_slots),ROOT/'worlds'/('random_'+stage+'.sdf'))
    save_xml(world(c,'audit',layouts['competition'],audit=True),ROOT/'worlds/audit_views.sdf')
    save_xml(world(c,'audit_stacking',layouts['competition'],audit=True),ROOT/'worlds/audit_stacking.sdf')
    save_xml(world(c,'audit_random',random_points,audit=True),ROOT/'worlds/audit_random.sdf')
    examples={
        'slots_debug':(random_points,randomization),
        'slots_all':([],{'selected_slots':[],'obstacle_positions_m':[],'seed':seed}),
    }
    for name,ids in [('central_slots',['R9','R10','R11','R12','R13']),('manual_slots',['R2','R7','R10'])]:
        pts,info,_,_=select_scene(c,settings,seed,mode='manual_slots',manual_slots=ids,validator=validator)
        examples[name]=(pts,info)
    pts,info,_,_=select_scene(c,settings,seed,mode='stress_slots',validator=validator)
    examples['stress_slots']=(pts,info)
    for name,(pts,info) in examples.items():
        layouts[name]=pts
        save_xml(world(c,name,pts,debug=True),ROOT/'worlds'/(name+'.sdf'))
        save_xml(world(c,name,pts,audit=True,debug=True),ROOT/'worlds'/('audit_'+name+'.sdf'))
    (ROOT/'config/generated_slot_examples.json').write_text(json.dumps({k:v[1] for k,v in examples.items()},indent=2)+'\n')
    (ROOT/'config/slot_validation.json').write_text(json.dumps(validate_slots(c,settings),indent=2)+'\n')
    save_xml(world(c,'audit_random_stacking',random_points,audit=True),ROOT/'worlds/audit_random_stacking.sdf')
    save_xml(world(c,'central_blocked',layouts['central_blocked'],audit=True),ROOT/'worlds/audit_central_blocked.sdf')
    if 'stress_6' in layouts:
        save_xml(world(c,'stress_6',layouts['stress_6'],audit=True),ROOT/'worlds/audit_stress_6.sdf')
    save_xml(world(c,'contact',[],probe=True),ROOT/'worlds/contact_test.sdf')
    build_maps(c,layouts)
    (ROOT/'maps/random_seeded_obstacles.json').write_text(json.dumps({
        'frame':'map','map_to_world':c['frame']['map_to_world'],'seed':seed,
        'resolution_m':c['map']['resolution'],'obstacles':randomization['obstacles'],
        'candidate_slots':slots},indent=2)+'\n')
    # Existing planner's ordered rectangle interface; static rules only, not live sensor data.
    regions=[[0.,0.,2.4,2.4,0.]]
    f=c['field']
    for x,y in f['yellow_lower_left']:
        regions.append([x,y,x+f['yellow_size'][0],y+f['yellow_size'][1],100.])
    for k in ['coarse','temporary']: regions.append([*box_bounds(c['stations'][k]),100.])
    regions.append([*c['stations']['raw']['exclusion_bounds'],-1.])
    regions.append([*details.board_bounds(c),100.])
    for start in f['starts']:
        x,y=start['lower_left']; a,b=f['start_size']; regions.append([x,y,x+a,y+b,100.])
    params={'simulation_mode':True,'output_prefix':'/gazebo_field_adapter','map_frame':'map',
        'map.layout':'custom','map.provenance':'OFFICIAL_PDF_RECONSTRUCTION_UNCALIBRATED',
        'map.confirmed':False,'frames_confirmed':False,'footprint.confirmed':False,
        'map.width':480,'map.height':480,'map.resolution':c['map']['resolution'],
        'map.origin_x':0.,'map.origin_y':0.,'map.regions':[float(v) for r in regions for v in r]}
    (ROOT/'config/planner_static_adapter.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':params}},sort_keys=False))
    hashes = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
              for folder in ['config','models','worlds','maps'] for p in sorted((ROOT/folder).rglob('*')) if p.is_file()}
    (ROOT/'generated_manifest.json').write_text(json.dumps(hashes,indent=2)+'\n',encoding='utf-8')
    print(f'Generated {len(hashes)} files from unified geometry, {len(layouts)} layouts.')
    print(f'Scene seed={seed}, obstacles={count}, attempts={randomization["attempts"]}, task={randomization["task_code"]}, status={assessment["status"]}')


if __name__ == '__main__':
    main()
