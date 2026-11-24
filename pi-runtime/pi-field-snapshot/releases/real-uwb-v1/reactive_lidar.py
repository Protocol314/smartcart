"""Reactive LiDAR geometry. Points/adjacent segments, no obstacle circles."""
import math
from dataclasses import dataclass

EPS=1e-9
def closest(p,a,b):
    vx,vy=b[0]-a[0],b[1]-a[1];den=vx*vx+vy*vy
    t=max(0.,min(1.,((p[0]-a[0])*vx+(p[1]-a[1])*vy)/den)) if den else 0.
    return a[0]+t*vx,a[1]+t*vy
def box_hit(a,b,box):
    lo,hi=0.,1.
    for p,q,mn,mx in ((a[0],b[0],box[0],box[1]),(a[1],b[1],box[2],box[3])):
        d=q-p
        if abs(d)<EPS:
            if p<mn or p>mx:return False
        else:
            t0,t1=sorted(((mn-p)/d,(mx-p)/d))
            lo,hi=max(lo,t0),min(hi,t1)
            if lo>hi:return False
    return True
def point_box(p,box):
    q=(max(box[0],min(box[1],p[0])),max(box[2],min(box[3],p[1])))
    return math.dist(p,q),p
def segment_box(a,b,box):
    if box_hit(a,b,box):return 0.,closest((0.,0.),a,b)
    options=[point_box(a,box),point_box(b,box)]
    for c in ((box[0],box[2]),(box[0],box[3]),(box[1],box[2]),(box[1],box[3])):
        p=closest(c,a,b);options.append((math.dist(c,p),p))
    return min(options,key=lambda v:v[0])
def local(p,angle):
    c,s=math.cos(angle),math.sin(angle)
    return p[0]*c+p[1]*s,-p[0]*s+p[1]*c

@dataclass(frozen=True)
class Record:
    clearance:float
    witness:tuple
    a:tuple
    b:tuple

@dataclass(frozen=True)
class Scene:
    records:tuple
    minimum:float
    rays:tuple
    primitives:tuple

def body_box(g,pad=0.):
    return (-g['rear_extent_m']-pad,g['front_extent_m']+pad,
            -g['width_m']/2-pad,g['width_m']/2+pad)

def build_scene(scan,g,join_gap=.15):
    box=body_box(g);near=body_box(g,.65)
    records=[];primitives=[];previous=None;first=None;best=math.inf
    # Scan.rays already contains the installed calibrated LiDAR yaw.
    for angle,r in scan.rays:
        if r is None:
            previous=None;continue
        a=math.radians(angle)
        p=(g['lidar_x_m']+r*math.cos(a),g['lidar_y_m']+r*math.sin(a))
        if first is None and len(primitives)==0 and angle==scan.rays[0][0]:first=p
        d,w=point_box(p,box);best=min(best,d)
        if near[0]<=p[0]<=near[1] and near[2]<=p[1]<=near[3]:
            records.append(Record(d,w,p,p));primitives.append((p,p))
        if previous is not None and math.dist(previous,p)<=join_gap:
            # Far endpoints may enclose a near segment: clip before pruning.
            if box_hit(previous,p,near):
                d,w=segment_box(previous,p,box);best=min(best,d)
                records.append(Record(d,w,previous,p));primitives.append((previous,p))
        previous=p
    delta=abs((scan.rays[1][0]-scan.rays[0][0]+180.)%360.-180.) if len(scan.rays)>1 else 0.
    full=len(scan.rays)>=180 and delta>0 and abs(len(scan.rays)*delta-360.)<=2*delta
    if full and first is not None and previous is not None and math.dist(previous,first)<=join_gap:
        if box_hit(previous,first,near):
            d,w=segment_box(previous,first,box);best=min(best,d)
            records.append(Record(d,w,previous,first));primitives.append((previous,first))
    return Scene(tuple(records),best,scan.rays,tuple(primitives))

def assess(scene,g,heading,forward,turn,rules):
    c,s=math.cos(heading),math.sin(heading)
    front=(g['front_extent_m'],0.)
    front_min=left=right=near=math.inf
    pivot=abs(forward)<.08 and abs(turn)>=.08
    for record in scene.records:
        a,b=record.a,record.b
        # Ignore objects wholly behind the rear when translating forward.
        if not pivot and max(a[0]*c+a[1]*s,b[0]*c+b[1]*s)<-g['rear_extent_m']:
            continue
        near=min(near,record.clearance)
        witness=closest(front,a,b)
        dx,dy=witness[0]-front[0],witness[1]-front[1]
        along=dx*c+dy*s;lateral=-dx*s+dy*c
        bearing=math.degrees(math.atan2(lateral,along))
        if along>=0 and abs(bearing)<=rules['front_half_deg']:
            front_min=min(front_min,record.clearance)
        # Only connected returns steer away from side walls.
        elif a != b:
            if lateral > 0:
                left = min(left, record.clearance)
            elif lateral < 0:
                right = min(right, record.clearance)
    def strength(d):
        return max(0.,min(1.,(rules['slow_m']-d)/(rules['slow_m']-rules['stop_m'])))
    def side_strength(gap):
        return max(0.0, min(1.0, (0.20 - gap) / 0.20))
    return dict(minimum=scene.minimum,near=near,front=front_min,
                left=left,right=right,scale=1.-strength(near),
                push=side_strength(right)-side_strength(left))

def unknown_sector(scene,forward,turn,front_half):
    if abs(forward)<.08 and abs(turn)<.08:return 0,0
    pivot=abs(forward)<.08 and abs(turn)>=.08
    required=[r for a,r in scene.rays if pivot or abs(a)<=front_half
              or turn>=.08 and front_half<=a<=135.
              or turn<=-.08 and -135.<=a<=-front_half]
    return len(required),sum(r is None for r in required)

def rotation_clear(scene,g,angle,margin):
    # Geometric preview only, NOT a calibrated stopping/rotation prediction.
    # <=1deg samples plus a corner-displacement bound cover intermediate poses.
    count=max(1,math.ceil(abs(angle)/math.radians(1.)))
    radius=math.hypot(max(g['front_extent_m'],g['rear_extent_m']),g['width_m']/2)
    extra=radius*abs(angle)/count/2
    box=body_box(g,margin+extra)
    for i in range(count+1):
        yaw=angle*i/count
        for a,b in scene.primitives:
            if box_hit(local(a,yaw),local(b,yaw),box):return False
    return True


# Directional visibility: inspect only rays intersecting newly swept body space.
def _cross(a,b):return a[0]*b[1]-a[1]*b[0]
def _hull(points):
    points=sorted(set(points))
    def half(items):
        out=[]
        for p in items:
            while len(out)>1 and _cross((out[-1][0]-out[-2][0],out[-1][1]-out[-2][1]),(p[0]-out[-1][0],p[1]-out[-1][1]))<=EPS:out.pop()
            out.append(p)
        return out
    return tuple(half(points)[:-1]+half(reversed(points))[:-1])
def _ray_interval(origin,direction,polygon):
    lo,hi=0.,100.
    for p,q in zip(polygon,polygon[1:]+polygon[:1]):
        e=(q[0]-p[0],q[1]-p[1])
        c=_cross(e,(origin[0]-p[0],origin[1]-p[1]));v=_cross(e,direction)
        if abs(v)<EPS:
            if c < -EPS:return None
        elif v>0:lo=max(lo,-c/v)
        else:hi=min(hi,-c/v)
        if lo>hi+EPS:return None
    return lo,hi
def motion_visibility(scene,g,forward,turn,heading,lookahead=.30,preview_deg=20.):
    if abs(forward)<.08 and abs(turn)<.08:return 0,0
    length=math.copysign(lookahead,forward) if abs(forward)>=.08 else 0.
    angle=0.
    if abs(turn)>=.08:
        angle=math.copysign(max(math.radians(1.),min(abs(heading),math.radians(preview_deg))),turn)
    n=max(1,math.ceil(abs(angle)/math.radians(2.)))
    box=body_box(g)
    body=((box[0],box[2]),(box[1],box[2]),(box[1],box[3]),(box[0],box[3]))
    radius=math.hypot(max(g['front_extent_m'],g['rear_extent_m']),g['width_m']/2)
    pad=radius*abs(angle)/n/2
    xlo,xhi=min(0.,length,length*math.cos(angle)),max(0.,length,length*math.cos(angle))
    ylo,yhi=min(0.,length*math.sin(angle)),max(0.,length*math.sin(angle))
    points=[]
    for i in range(n+1):
        a=angle*i/n;c,s=math.cos(a),math.sin(a)
        for x,y in body:
            for dx in (xlo,xhi):
                for dy in (ylo,yhi):
                    for px,py in ((-pad,-pad),(-pad,pad),(pad,-pad),(pad,pad)):
                        points.append((dx+x*c-y*s+px,dy+x*s+y*c+py))
    swept=_hull(points);origin=(g['lidar_x_m'],g['lidar_y_m'])
    return _angular_unknown(scene,swept,origin,box)

def rear_clear(scene,g,rules,visibility=None):
    checked,unknown=(motion_visibility(scene,g,-1.,0.,0.,rules['reverse_lookahead_m']) if visibility is None else visibility)
    # Missing returns alone do not stop motion.
    margin=rules['stop_m']
    box=(-g['rear_extent_m']-rules['reverse_lookahead_m']-margin,
         g['front_extent_m']+margin,-g['width_m']/2-margin,g['width_m']/2+margin)
    if any(box_hit(a,b,box) for a,b in scene.primitives):
        return False,'reactive_recovery_rear_occupied'
    return True,'reactive_reverse_clear'


def _clip_half(polygon,axis,bound,sign):
    if not polygon:return ()
    out=[]
    a=polygon[-1];va=sign*(a[axis]-bound);ina=va>=0.
    for b in polygon:
        vb=sign*(b[axis]-bound);inb=vb>=0.
        if ina!=inb:
            t=va/(va-vb)
            out.append((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])))
        if inb:out.append(b)
        a,va,ina=b,vb,inb
    return tuple(out)

def _angular_unknown(scene,swept,origin,box):
    # Outside-current-body pieces are convex. Their angular intervals tell
    # exactly which rays can enter NEW swept space, without per-ray clipping.
    arcs=[]
    for axis,bound,sign in ((0,box[0]-1e-9,-1),(0,box[1]+1e-9,1),
                            (1,box[2]-1e-9,-1),(1,box[3]+1e-9,1)):
        p=_clip_half(swept,axis,bound,sign)
        if len(p)<3:continue
        area=sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(p,p[1:]+p[:1]))
        if abs(area)<1e-16:continue
        inside=all(_cross((b[0]-a[0],b[1]-a[1]),(origin[0]-a[0],origin[1]-a[1]))>1e-10
                   for a,b in zip(p,p[1:]+p[:1]))
        if inside:
            return len(scene.rays),sum(r is None for a,r in scene.rays)
        angles=sorted(math.degrees(math.atan2(y-origin[1],x-origin[0]))%360.
                      for x,y in p if math.hypot(x-origin[0],y-origin[1])>1e-12)
        if not angles:continue
        gaps=[(angles[(i+1)%len(angles)]-angles[i])%360. for i in range(len(angles))]
        i=max(range(len(gaps)),key=gaps.__getitem__)
        start=angles[(i+1)%len(angles)];width=360.-gaps[i]
        if width>180.+1e-7:
            return len(scene.rays),sum(r is None for a,r in scene.rays)
        arcs.append((start,width))
    checked=unknown=0
    for a,r in scene.rays:
        if any(((a-start)%360.)<=width+1e-8 or ((a-start)%360.)>=360.-1e-8
               for start,width in arcs):
            checked+=1;unknown+=r is None
    return checked,unknown

