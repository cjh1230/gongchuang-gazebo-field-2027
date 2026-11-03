"""Seeded scene sampling and an offline, footprint-aware solvability certificate.

This is a scene acceptance check. It publishes no robot commands or navigation topics.
"""
import heapq
import itertools
import math
import random


def seeded_task(seed):
    rng=random.Random(f'gongchuang-colors-v1:{seed}')
    colors=rng.sample(range(1,7),3)
    groups=[colors,rng.sample([1,2,3],3),rng.sample(colors,3),rng.sample([1,2,3],3)]
    return '+'.join(''.join(map(str,g)) for g in groups)


def station_bounds(s):
    x,y=s['center']; a,b=s['size']; angle=s['yaw']
    hx=(abs(a*math.cos(angle))+abs(b*math.sin(angle)))/2
    hy=(abs(a*math.sin(angle))+abs(b*math.cos(angle)))/2
    return [x-hx,y-hy,x+hx,y+hy]


def forbidden(c,allow_start=None):
    f=c['field']; rects=[]
    for x,y in f['yellow_lower_left']: rects.append([x,y,x+f['yellow_size'][0],y+f['yellow_size'][1]])
    for s in f['starts']:
        if s['id']!=allow_start:
            x,y=s['lower_left']; a,b=f['start_size']; rects.append([x,y,x+a,y+b])
    rects.extend(station_bounds(c['stations'][k]) for k in ['coarse','temporary'])
    rects.append(c['stations']['raw']['exclusion_bounds'])
    q=c['qr_board']; x,y,_=q['center']; a=q['thickness']; b=q['size'][0]; angle=q['yaw']
    hx=(abs(a*math.cos(angle))+abs(b*math.sin(angle)))/2
    hy=(abs(a*math.sin(angle))+abs(b*math.cos(angle)))/2
    rects.append([x-hx,y-hy,x+hx,y+hy])
    return rects


def overlap(a,b):
    return a[0]<b[2]-1e-10 and a[2]>b[0]+1e-10 and a[1]<b[3]-1e-10 and a[3]>b[1]+1e-10


def circle_hits_box(x,y,r,b):
    dx=max(b[0]-x,0,x-b[2]); dy=max(b[1]-y,0,y-b[3])
    return dx*dx+dy*dy <= r*r+1e-12


def legal_obstacle(c,x,y):
    """The complete cylinder footprint must lie in the configured official gray region."""
    radius=c['obstacle']['diameter']/2
    return (radius<=x<=c['field']['size'][0]-radius and radius<=y<=c['field']['size'][1]-radius
        and not any(circle_hits_box(x,y,radius,box) for box in forbidden(c)))


def legal_layout(c,settings,points):
    return all(legal_obstacle(c,x,y) and all(math.hypot(x-a,y-b)>=settings['minimum_obstacle_separation']
        for a,b in points[:i]) for i,(x,y) in enumerate(points))


def sweep_clear(a,b,half,c,obstacles,rects):
    if abs(a[0]-b[0])>1e-9 and abs(a[1]-b[1])>1e-9: return False
    box=[min(a[0],b[0])-half,min(a[1],b[1])-half,max(a[0],b[0])+half,max(a[1],b[1])+half]
    if box[0]<-1e-9 or box[1]<-1e-9 or box[2]>c['field']['size'][0]+1e-9 or box[3]>c['field']['size'][1]+1e-9: return False
    return not any(overlap(box,r) for r in rects) and not any(circle_hits_box(x,y,c['obstacle']['diameter']/2,box) for x,y in obstacles)


def goals_and_departures(c,settings):
    v=settings['validation']; goals={'raw_material':v['raw_approach'],'qr_board':v['qr_approach']}
    for kind in ['coarse','temporary']:
        station=c['stations'][kind]; angle=station['yaw']; x,y=station['center']
        normal=[0,1] if kind=='coarse' else [1,0]
        for slot,lx in c['targets']['slot_local_x'].items():
            goals[f'{kind}_slot_{slot}']=[round(x+lx*math.cos(angle)+normal[0]*v['station_approach_offset'],2),
                round(y+lx*math.sin(angle)+normal[1]*v['station_approach_offset'],2)]
    departures={}
    for start in c['field']['starts']:
        sid=start['id']; x,y=start['lower_left']; a,b=c['field']['start_size']
        center=[x+a/2,y+b/2]; port=v[sid+'_port']
        route=[center,[port[0],center[1]],port]
        departures[sid]=route; goals[sid+'_port']=port
    return goals,departures


def footprint_half(settings, include_grid=False):
    """Envelope of the actual square over all allowed heading errors, plus budgets."""
    v=settings['validation']; a=math.radians(v['heading_error_deg'])
    return v['robot_square_side']/2*(math.cos(a)+math.sin(a))+v['safety_margin']+v['tracking_error']+(v['edge_padding'] if include_grid else 0)


def validate_slots(c,settings):
    slots=settings['candidate_slots']['slots']; errors=[]; report=[]
    if set(slots)!={f'R{i}' for i in range(1,14)}: raise ValueError('Candidate IDs must be exactly R1..R13')
    for sid,e in slots.items():
        pt=e['xy_m']
        if len(pt)!=2 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in pt):
            errors.append(f'{sid}: malformed/nonfinite coordinate'); continue
        valid=legal_obstacle(c,*pt)
        if not valid: errors.append(f'{sid}: {pt} intersects facility/forbidden region or boundary')
        report.append({'slot_id':sid,'xy_m':pt,'legal':valid,'relation':e['relation']})
    for a,b in itertools.combinations(slots,2):
        if math.dist(slots[a]['xy_m'],slots[b]['xy_m'])<settings['minimum_obstacle_separation']:
            errors.append(f'{a}/{b}: duplicate or insufficiently separated')
    if errors: raise ValueError('Invalid slots; no positions moved:\n'+'\n'.join(errors))
    return report


class Validator:
    def __init__(self,c,settings):
        self.c=c; self.settings=settings; v=settings['validation']
        self.step=v['grid_step']; self.half=v['robot_square_side']/2; self.padding=v['edge_padding']
        self.sweep_half=footprint_half(settings)
        if self.step<=0 or self.half<=0 or v['heading_rad']!=0: raise ValueError('Positive square footprint and fixed heading zero required')
        if self.padding+1e-10<self.step/2: raise ValueError('Edge padding must cover half a grid step')
        self.n=round(c['field']['size'][0]/self.step)+1; self.ny=round(c['field']['size'][1]/self.step)+1
        self.rects=forbidden(c); self.goals,self.departures=goals_and_departures(c,settings)
        if any(settings['navigation_task'][k] not in self.goals for k in ('start','goal')): raise ValueError('Unknown task endpoint')
        self.base=bytearray(self.n*self.ny); h=footprint_half(settings,True)
        for j in range(self.ny):
            for i in range(self.n):
                p=[i*self.step,j*self.step]; self.base[j*self.n+i]=sweep_clear(p,p,h,c,[],self.rects)

    def _free_grid(self,points):
        c=self.c; n=self.n; h=footprint_half(self.settings,True); free=self.base.copy(); r=c['obstacle']['diameter']/2
        for x,y in points:
            for j in range(max(0,int((y-h-r)/self.step)),min(self.ny,math.ceil((y+h+r)/self.step)+1)):
                for i in range(max(0,int((x-h-r)/self.step)),min(n,math.ceil((x+h+r)/self.step)+1)):
                    if circle_hits_box(x,y,r,[i*self.step-h,j*self.step-h,i*self.step+h,j*self.step+h]): free[j*n+i]=0
        return free

    def route_status(self,points,start,goal,free=None):
        """Fixed-heading four-neighbor A*, followed by full continuous rectangle sweeps."""
        free=self._free_grid(points) if free is None else free; n=self.n; cells=[]
        for pt in (start,goal):
            if not all(math.isfinite(v) for v in pt): raise ValueError('Nonfinite endpoint')
            ix,iy=round(pt[0]/self.step),round(pt[1]/self.step)
            if not (0<=ix<n and 0<=iy<self.ny): return {'status':'NO_PATH','reason':'ENDPOINT_OUTSIDE_GRID'}
            if max(abs(pt[0]-ix*self.step),abs(pt[1]-iy*self.step))>1e-8:
                raise ValueError('Endpoints must align to validation grid; no silent snapping')
            cells.append(iy*n+ix)
        source,target=cells
        if not free[source] or not free[target]: return {'status':'NO_PATH','reason':'ENDPOINT_BLOCKED'}
        def heuristic(idx): return abs(idx%n-target%n)+abs(idx//n-target//n)
        queue=[(heuristic(source),0,source)]; costs={source:0}; previous={source:source}
        while queue:
            _,cost,cur=heapq.heappop(queue)
            if cost!=costs[cur]: continue
            if cur==target:
                seq=[cur]
                while seq[-1]!=source: seq.append(previous[seq[-1]])
                path=[]
                for idx in reversed(seq):
                    pt=[round(idx%n*self.step,6),round(idx//n*self.step,6)]
                    while len(path)>1 and (path[-2][0]==path[-1][0]==pt[0] or path[-2][1]==path[-1][1]==pt[1]): path.pop()
                    path.append(pt)
                if not sweep_clear(start,start,self.sweep_half,self.c,points,self.rects) or not all(
                    sweep_clear(a,b,self.sweep_half,self.c,points,self.rects) for a,b in zip(path,path[1:])):
                    raise AssertionError('Unsafe A* sweep')
                return {'status':'PATH','reason':'CONNECTED','path_m':path,'expanded':len(previous)}
            x,y=cur%n,cur//n
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                xx,yy=x+dx,y+dy; nxt=yy*n+xx
                if 0<=xx<n and 0<=yy<self.ny and free[nxt] and cost+1<costs.get(nxt,math.inf):
                    costs[nxt]=cost+1; previous[nxt]=cur; heapq.heappush(queue,(cost+1+heuristic(nxt),cost+1,nxt))
        return {'status':'NO_PATH','reason':'DISCONNECTED','expanded':len(previous)}

    def certify(self,points,include_paths=True):
        task=self.settings['navigation_task']; start=self.goals[task['start']]; goal=self.goals[task['goal']]
        route=self.route_status(points,start,goal)
        if route['status']!='PATH': return None
        v=self.settings['validation']
        cert={'status':'pass','method':'fixed_heading_A_star_continuous_robust_rectangle_sweep',
            'robot_square_side_m':self.half*2,'heading_rad':0,'grid_step_m':self.step,'edge_padding_m':self.padding,
            'safety_margin_m':v['safety_margin'],'tracking_error_m':v['tracking_error'],'heading_error_deg':v['heading_error_deg'],
            'sweep_half_m':self.sweep_half,'task':dict(task),'start_m':start,'goal_m':goal,
            'scope':'configured_gray_navigation_task_only; blue_docking_separate'}
        if include_paths: cert['path_m']=route['path_m']
        return cert

    def assess(self,points):
        if not legal_layout(self.c,self.settings,points): return {'status':'INVALID_PLACEMENT','obstacle_count':len(points)}
        cert=self.certify(points); v=self.settings['validation']; task=self.settings['navigation_task']
        raw=self.goals['raw_material']; coarse=self.goals['coarse_slot_2']
        return {'status':'PATH' if cert else 'NO_PATH','obstacle_count':len(points),'navigation_task':dict(task),
            'robot_footprint_m':[self.half*2,self.half*2],'edge_padding_m':self.padding,
            'safety_margin_m':v['safety_margin'],'tracking_error_m':v['tracking_error'],'heading_error_deg':v['heading_error_deg'],
            'robust_sweep_half_m':self.sweep_half,'task_result':self.route_status(points,self.goals[task['start']],self.goals[task['goal']]),
            'raw_to_coarse':{'start':raw,'goal':coarse,**self.route_status(points,raw,coarse)},
            'blue_zone_policy':'transit_forbidden; selected_departure_nominal_docking_checked_separately',
            'nominal_blue_departures':{sid:all(sweep_clear(a,b,self.half,self.c,points,forbidden(self.c,sid))
                for a,b in zip(path,path[1:])) for sid,path in self.departures.items()},'certificate':cert}


def verify_certificate(c,settings,points,cert):
    if not cert or cert.get('status')!='pass' or not legal_layout(c,settings,points): return False
    task=settings['navigation_task']; goals,_=goals_and_departures(c,settings); v=settings['validation']
    start,goal=goals[task['start']],goals[task['goal']]
    required={'task':task,'start_m':start,'goal_m':goal,'robot_square_side_m':v['robot_square_side'],
        'sweep_half_m':footprint_half(settings),'safety_margin_m':v['safety_margin'],'tracking_error_m':v['tracking_error'],
        'heading_error_deg':v['heading_error_deg'],'heading_rad':v['heading_rad'],'grid_step_m':v['grid_step'],'edge_padding_m':v['edge_padding']}
    if any(cert.get(k)!=val for k,val in required.items()): return False
    path=cert.get('path_m',[])
    if not path or path[0]!=start or path[-1]!=goal: return False
    half=footprint_half(settings); rects=forbidden(c)
    return sweep_clear(start,start,half,c,points,rects) and all(sweep_clear(a,b,half,c,points,rects) for a,b in zip(path,path[1:]))


MODES=('random_slots','solvable_slots','blocked_slots','stress_slots','manual_slots')


def draw_ids(settings,seed,count=None,mode='random_slots',manual_slots=None):
    """Pure unfiltered lottery; separate RNG from material/QR task colors."""
    if type(seed) is not int or not 0<=seed<2**32: raise ValueError('Seed must be uint32')
    if mode not in MODES: raise ValueError('Unknown slot mode')
    rng=random.Random(f'gongchuang-slots-v2:{seed}'); available=list(settings['candidate_slots']['slots'])
    if mode=='manual_slots':
        ids=list(manual_slots or [])
        if len(ids)!=len(set(ids)): raise ValueError('Duplicate manual slot ID')
        if any(s not in available for s in ids): raise ValueError('Unknown manual slot ID')
        if count is not None and count!=len(ids): raise ValueError('Manual count conflicts with IDs')
        return ids
    if manual_slots is not None: raise ValueError('Manual IDs require manual_slots mode')
    if count is None: count=rng.randint(3,6) if mode=='stress_slots' else settings['obstacle_count']
    if type(count) is not int or not 0<=count<=13: raise ValueError('Obstacle count must be 0..13')
    if mode=='stress_slots' and not 3<=count<=6: raise ValueError('Stress count must be 3..6')
    return rng.sample(available,count)


def select_scene(c,settings,seed,count=None,mode=None,manual_slots=None,validator=None):
    mode=mode or settings['mode']; validate_slots(c,settings)
    if mode=='blocked_slots' and count is None: count=len(settings['presets']['central_blocked']['slots'])
    ids=draw_ids(settings,seed,count,mode,manual_slots)
    slots=settings['candidate_slots']['slots']; validator=validator or Validator(c,settings); attempts=0
    if mode in ('solvable_slots','blocked_slots'):
        combos=list(itertools.combinations(list(slots),len(ids)))
        random.Random(f'gongchuang-filter-v2:{seed}').shuffle(combos)
        wanted='PATH' if mode=='solvable_slots' else 'NO_PATH'
        for combination in combos:
            attempts+=1; ids=list(combination); points=[list(slots[s]['xy_m']) for s in ids]; result=validator.assess(points)
            if result['status']==wanted and (mode!='blocked_slots' or result['task_result']['reason']=='DISCONNECTED'): break
        else: raise RuntimeError(f'No {wanted} combination for count={len(ids)} and task; no positions changed')
    else:
        attempts=1; points=[list(slots[s]['xy_m']) for s in ids]; result=validator.assess(points)
    cert=result['certificate']
    if cert and not verify_certificate(c,settings,points,cert): raise AssertionError('Invalid path certificate')
    info={'seed':seed,'mode':mode,'obstacle_count':len(ids),'selected_slots':ids,'obstacle_positions_m':points,
        'obstacles':[{'entity_name':f'obstacle_{i+1}','slot_id':sid,'position_m':[*pt,0],
            'diameter_m':c['obstacle']['diameter'],'height_m':c['obstacle']['height']} for i,(sid,pt) in enumerate(zip(ids,points))],
        'attempts':attempts,'task_code':seeded_task(seed),'algorithm_version':'gongchuang_slots_v2',
        'status':'reference_image_uncalibrated_test_draw','classification':'development_test_not_official_quantity',
        'reachability_status':result['status'],'navigation_task':settings['navigation_task'],
        'candidate_coordinate_status':settings['candidate_slots']['coordinate_status']}
    return points,info,cert,result


def sample_scene(c,settings,seed,count,validator=None):
    """Compatibility API: filter finite slot combinations for the configured navigation task."""
    points,info,cert,_=select_scene(c,settings,seed,count,'solvable_slots',validator=validator)
    return points,info,cert


def preset_scene(c,settings,seed,preset,validator=None):
    entry=settings['presets'][preset]
    points,info,cert,result=select_scene(c,settings,seed,entry.get('obstacle_count'),
        'manual_slots' if 'slots' in entry else entry['mode'],entry.get('slots'),validator)
    if entry['mode']=='blocked_slots' and result['status']!='NO_PATH': raise ValueError('Blocked preset no longer blocks task; no positions moved')
    info.update({'preset':preset,'mode':entry['mode'],'classification':entry['classification']})
    return points,info,cert,result
