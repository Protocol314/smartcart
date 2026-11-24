"""Point/segment local routes and conservative rectangular swept-space checks."""
import heapq
import math
import time
from dataclasses import dataclass

EPS=1e-7
def cross(a,b):return a[0]*b[1]-a[1]*b[0]
def sub(a,b):return (a[0]-b[0],a[1]-b[1])
def dist(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])
def nearest(p,a,b):
    v=sub(b,a);den=v[0]**2+v[1]**2
    t=max(0.,min(1.,sum(x*y for x,y in zip(sub(p,a),v))/den)) if den else 0.
    return a[0]+v[0]*t,a[1]+v[1]*t
def hull(points):
    points=sorted(set(points))
    def half(items):
        out=[]
        for p in items:
            while len(out)>1 and cross(sub(out[-1],out[-2]),sub(p,out[-1]))<=EPS:out.pop()
            out.append(p)
        return out
    return tuple(half(points)[:-1]+half(reversed(points))[:-1])
@dataclass(frozen=True)
class Polygon:
    vertices:tuple
    kind:str='segment'
    def clearance(self,p):
        edges=tuple(zip(self.vertices,self.vertices[1:]+self.vertices[:1]))
        d=min(dist(p,nearest(p,a,b)) for a,b in edges)
        return -d if all(cross(sub(b,a),sub(p,a))>=-EPS for a,b in edges) else d
    def blocks(self,a,b):
        lo,hi=0.,1.;d=sub(b,a)
        for p,q in zip(self.vertices,self.vertices[1:]+self.vertices[:1]):
            e=sub(q,p);c=cross(e,sub(a,p));v=cross(e,d)
            if abs(v)<EPS:
                if c<=EPS:return False
            elif v>0:lo=max(lo,(EPS-c)/v)
            else:hi=min(hi,(EPS-c)/v)
            if lo>=hi:return False
        return lo<hi and hi>0 and lo<1

def primitives(scan,yaw,g,end):
    groups=[];group=[];step=scan['angle_increment']
    ranges=scan['ranges'];amin=scan['angle_min']
    for i,r in enumerate(ranges):
        if time.monotonic()>=end:raise TimeoutError('segment_budget')
        if type(r) not in (float,int) or not math.isfinite(r) or not scan['range_min']<=r<=scan['range_max']:
            if group:groups.append(group);group=[]
            continue
        a=amin+i*step+math.radians(yaw)
        p=(g['lidar_x_m']+r*math.cos(a),g['lidar_y_m']+r*math.sin(a))
        if group and dist(group[-1],p)>.15:
            groups.append(group);group=[]
        group.append(p)
    if group:groups.append(group)
    shapes=[]
    def split(points):
        if time.monotonic()>=end:raise TimeoutError('segment_budget')
        a,b=points[0],points[-1]
        errors=[dist(p,nearest(p,a,b)) for p in points]
        k=max(range(len(errors)),key=errors.__getitem__)
        if len(points)>2 and errors[k]>.01:
            split(points[:k+1]);split(points[k:])
        else:
            # Keep points and line segments. Residual bounds line-fit error;
            # no obstacle centre/radius or enclosing circle is constructed.
            shapes.append(('point' if len(points)==1 else 'segment',a,b,max(errors)))
    for group in groups:split(group)
    return tuple(shapes)

def cspace(shapes,g,footprint=None,extra=0.):
    body=footprint or ((-g['rear_extent_m'],-g['width_m']/2),
                       (g['front_extent_m'],-g['width_m']/2),
                       (g['front_extent_m'],g['width_m']/2),
                       (-g['rear_extent_m'],g['width_m']/2))
    out=[]
    for kind,a,b,residual in shapes:
        pad=.20+residual+extra
        offsets=((-pad,-pad),(-pad,pad),(pad,-pad),(pad,pad))
        out.append(Polygon(hull((p[0]-v[0]+d[0],p[1]-v[1]+d[1])
                               for p in (a,b) for v in body for d in offsets),kind))
    return tuple(out)

def route(goal,polygons,end,preferred=0):
    origin=(0.,0.)
    if any(p.clearance(origin)<=EPS for p in polygons):return (), 'segment_body_overlap'
    if any(p.clearance(goal)<=EPS for p in polygons):return (), 'segment_goal_occupied'
    def clear(a,b):return not any(p.blocks(a,b) for p in polygons)
    if clear(origin,goal):return (origin,goal),'direct'
    nodes=[origin,goal]
    for p in polygons:
        center=(sum(v[0] for v in p.vertices)/len(p.vertices),sum(v[1] for v in p.vertices)/len(p.vertices))
        for v in p.vertices:
            if time.monotonic()>=end:raise TimeoutError('segment_budget')
            # Stand slightly outside padded corners, never tangent to raw wall.
            dv=sub(v,center);d=max(dist(v,center),EPS)
            q=(v[0]+.015*dv[0]/d,v[1]+.015*dv[1]/d)
            if dist(origin,q)<4. and all(z.clearance(q)>EPS for z in polygons):nodes.append(q)
    nodes=list(dict.fromkeys(nodes))
    if len(nodes)>240:return (), 'segment_scene_complexity'
    queue=[(dist(origin,goal),0.,0)];cost={0:0.};prev={};done=set()
    while queue:
        if time.monotonic()>=end:raise TimeoutError('segment_budget')
        _,value,i=heapq.heappop(queue)
        if i in done:continue
        done.add(i)
        if i==1:
            ids=[1]
            while ids[-1]:ids.append(prev[ids[-1]])
            return tuple(nodes[k] for k in reversed(ids)),'detour'
        for j,p in enumerate(nodes):
            if time.monotonic()>=end:raise TimeoutError('segment_budget')
            if j in done:continue
            candidate=value+dist(nodes[i],p)+(.1 if i==0 and preferred*p[1]<0 else 0.)
            if candidate<cost.get(j,math.inf) and clear(nodes[i],p):
                cost[j]=candidate;prev[j]=i
                heapq.heappush(queue,(candidate+dist(p,goal),candidate,j))
    return (), 'segment_no_route'

def visible_disk(scan,g,yaw,x,y,radius):
    # Cover each grid cell by a disk solely for free-space proof.
    # No obstacle or route is modelled/drawn as a circle.
    if (-g['rear_extent_m']+radius<=x<=g['front_extent_m']-radius
            and abs(y)+radius<=g['width_m']/2):return True
    dx=x-g['lidar_x_m'];dy=y-g['lidar_y_m'];d=math.hypot(dx,dy)
    values=scan['ranges'];step=scan['angle_increment'];amin=scan['angle_min']
    full=abs(len(values)*abs(step)-2*math.pi)<=abs(step)*1.1
    def valid(r):return type(r) in (float,int) and math.isfinite(r) and scan['range_min']<=r<=scan['range_max'] and r>d+radius
    if d<=radius:return full and all(valid(r) for r in values)
    angle=math.atan2(dy,dx)-math.radians(yaw);spread=math.asin(min(1.,radius/d))
    for k in (-1,0,1):
        lo,hi=sorted(((angle+k*2*math.pi-spread-amin)/step,
                     (angle+k*2*math.pi+spread-amin)/step))
        i,j=math.floor(lo),math.ceil(hi)
        if not full and (i<0 or j>=len(values)):continue
        if all(valid(values[v%len(values)]) for v in range(i,j+1)):return True
    return False

def reachable_clear(scan,shapes,g,yaw,limits,age,end):
    # Bounds are measured at <=PWM40, inclusive of motor ramp/loads/slip.
    # Include scan age, next heartbeat, communication latency and full stop.
    # No wheel-speed feedback => consider BOTH turn directions and translations.
    duration=age+limits['reaction_s']+limits['stop_time_s']+.08
    travel=limits['max_speed_mps']*duration
    angle=min(math.pi,limits['max_yaw_rps']*duration)
    radius=math.hypot(max(g['front_extent_m'],g['rear_extent_m']),g['width_m']/2)
    count=max(1,math.ceil(angle/math.radians(2)))
    step=2*angle/count
    interp=radius*abs(step)/2
    noise=limits['position_error_m']+limits['object_speed_mps']*duration+interp
    corners=[]
    for i in range(count+1):
        a=-angle+step*i;c,s=math.cos(a),math.sin(a)
        for x in (-g['rear_extent_m'],g['front_extent_m']):
            for y in (-g['width_m']/2,g['width_m']/2):
                for dx,dy in ((-travel,-travel),(-travel,travel),(travel,-travel),(travel,travel)):
                    corners.append((dx+x*c-y*s,dy+x*s+y*c))
    footprint=hull(corners)
    if any(p.clearance((0.,0.))<=EPS for p in cspace(shapes,g,footprint,noise)):return False
    # Conservative bounding box coverage includes polygon interior and padding.
    pad=.20+noise
    xmin=min(p[0] for p in footprint)-pad;xmax=max(p[0] for p in footprint)+pad
    ymin=min(p[1] for p in footprint)-pad;ymax=max(p[1] for p in footprint)+pad
    nx=max(1,math.ceil((xmax-xmin)/.12));ny=max(1,math.ceil((ymax-ymin)/.12))
    sx=(xmax-xmin)/nx;sy=(ymax-ymin)/ny;r=math.hypot(sx,sy)/2
    for i in range(nx):
        if time.monotonic()>=end:raise TimeoutError('segment_budget')
        for j in range(ny):
            if not visible_disk(scan,g,yaw,xmin+(i+.5)*sx,ymin+(j+.5)*sy,r):return False
    return True

