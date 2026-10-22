"""Python 2/3 semantic navigation goals; nearest means shortest reachable path."""
from __future__ import division
import json
import math
import os


def patrol_goals(policy):
    route=policy.layout.get('preferred_patrol',{}).get('route_file')
    if route:
        if os.path.basename(route)!=route:raise ValueError('route must be scene-relative')
        with open(os.path.join(policy.folder,route)) as f:items=json.load(f)
        result=[]
        for item in items:
            point=list(item['point'])
            if len(point)!=2 or not policy.allowed(*point):raise ValueError('invalid patrol point')
            if not any(point==p for _,p in result):result.append((str(item['name']),point))
        if not 1<=len(result)<=12:raise ValueError('invalid patrol route length')
        return result
    return [('patrol_%d'%i,list(p)) for i,p in enumerate(policy.patrol_points(.70))]


def path_length(path):
    return sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:]))


def nearest_patrol(policy,current):
    candidates=[]
    for i,(label,point) in enumerate(patrol_goals(policy)):
        try:path=policy.plan(current,point)
        except ValueError:continue
        candidates.append((path_length(path),i,label,point,path))
    if not candidates:raise ValueError('no reachable patrol point from current pose')
    distance,index,label,point,path=min(candidates)
    return {'index':index,'label':label,'point':point,'path':path,'path_length_m':distance}


def nearest_route_entry(policy,current):
    route=patrol_goals(policy);candidates=[]
    for i,((label,a),(_,b)) in enumerate(zip(route,route[1:]+route[:1])):
        dx,dy=b[0]-a[0],b[1]-a[1];length2=dx*dx+dy*dy
        if length2<1e-9:continue
        t=max(0.,min(1.,((current[0]-a[0])*dx+(current[1]-a[1])*dy)/length2))
        point=[a[0]+t*dx,a[1]+t*dy]
        try:path=policy.plan(current,point)
        except ValueError:continue
        next_index=(i+1)%len(route)
        if t>=1.-1e-9:next_index=(i+2)%len(route);label=route[(i+1)%len(route)][0];point=route[(i+1)%len(route)][1]
        elif t>1e-9:label='join_'+label
        else:point=a
        candidates.append((path_length(path),i,{'label':label,'point':point,'next_index':next_index,'path':path}))
    if not candidates:raise ValueError('no reachable entry to patrol route')
    return min(candidates,key=lambda x:(x[0],x[1]))[2]


def resolve_destination(policy,current,destination):
    if destination=='dropoff-preparation':
        prep=policy.layout['bin'].get('navigation_preparation',{})
        point=prep.get('point_map')
        if point is None:raise ValueError('dropoff preparation point missing')
        policy.plan(current,point)
        return [('dropoff_preparation',list(point))]
    if destination=='nearest-patrol':
        nearest=nearest_patrol(policy,current)
        return [(nearest['label'],nearest['point'])]
    if destination=='patrol-cycle':
        route=patrol_goals(policy);entry=nearest_route_entry(policy,current);i=entry['next_index']
        first=(entry['label'],entry['point'])
        rotated=[first]+[item for item in route[i:]+route[:i] if item[1]!=first[1]]+[first]
        for a,b in zip(rotated,rotated[1:]):policy.plan(a[1],b[1])
        return rotated
    raise ValueError('unknown destination')
