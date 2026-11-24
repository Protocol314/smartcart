"""Local shortest paths around inflated, disjoint discs. No GPIO or ROS imports."""
from dataclasses import dataclass
import heapq
import math

EPS = 1e-8
TAU = 2 * math.pi

def add(a,b): return (a[0]+b[0],a[1]+b[1])
def sub(a,b): return (a[0]-b[0],a[1]-b[1])
def mul(a,s): return (a[0]*s,a[1]*s)
def norm(a): return math.hypot(*a)
def dist(a,b): return norm(sub(a,b))
def dot(a,b): return a[0]*b[0]+a[1]*b[1]

@dataclass(frozen=True)
class Disc:
    center: tuple
    radius: float

    def __post_init__(self):
        if len(self.center)!=2 or not all(math.isfinite(x) for x in self.center) or not math.isfinite(self.radius) or self.radius<=0:
            raise ValueError('invalid_disc')

@dataclass(frozen=True)
class Leg:
    start: tuple
    end: tuple
    length: float
    center: tuple = None
    radius: float = 0.
    sweep: float = 0.

    def at(self, distance):
        fraction=min(1.,max(0.,distance/max(EPS,self.length)))
        if self.center is None:
            return add(self.start,mul(sub(self.end,self.start),fraction))
        a=math.atan2(self.start[1]-self.center[1],self.start[0]-self.center[0])+self.sweep*fraction
        return add(self.center,(self.radius*math.cos(a),self.radius*math.sin(a)))

@dataclass(frozen=True)
class Route:
    legs: tuple
    length: float
    zones: tuple

    def at(self,distance):
        for leg in self.legs:
            if distance<=leg.length:return leg.at(distance)
            distance-=leg.length
        return self.legs[-1].end if self.legs else (0.,0.)

    def as_dict(self):
        return {'length_m':self.length,'zones':[{'center':d.center,'radius':d.radius} for d in self.zones],
                'legs':[{'kind':'arc' if l.center is not None else 'line','start':l.start,'end':l.end,
                         'length_m':l.length,'center':l.center,'radius':l.radius,'sweep_rad':l.sweep} for l in self.legs]}

def enclosing_zone(points, inflation, noise=.03):
    """Endpoint-diameter disc, expanded if a nonstraight cluster protrudes."""
    if not points or inflation<0 or noise<0:raise ValueError('invalid_cluster')
    center=mul(add(points[0],points[-1]),.5)
    radius=max(dist(points[0],points[-1])*.5,*(dist(p,center) for p in points))
    return Disc(center,radius+inflation+noise)

def merge_zones(zones):
    """Conservatively replace touching discs by their minimum enclosing disc."""
    result=list(zones)
    while True:
        merged=False
        for i,a in enumerate(result):
            for j in range(i+1,len(result)):
                b=result[j];d=dist(a.center,b.center)
                if d>a.radius+b.radius+EPS:continue
                if a.radius>=d+b.radius:new=a
                elif b.radius>=d+a.radius:new=b
                else:
                    radius=(d+a.radius+b.radius)*.5
                    new=Disc(add(a.center,mul(sub(b.center,a.center),(radius-a.radius)/d)),radius)
                result=[c for k,c in enumerate(result) if k not in (i,j)]+[new]
                merged=True;break
            if merged:break
        if not merged:return tuple(result)

def segment_clear(a,b,zones):
    v=sub(b,a);length2=dot(v,v)
    for z in zones:
        t=0. if length2<EPS*EPS else max(0.,min(1.,dot(sub(z.center,a),v)/length2))
        if dist(add(a,mul(v,t)),z.center)<z.radius-EPS:return False
    return True

def point_tangencies(point,z):
    v=sub(point,z.center);d=norm(v)
    if d<z.radius-EPS:return []
    phi=math.atan2(v[1],v[0]);offset=math.acos(min(1.,z.radius/d))
    return [add(z.center,(z.radius*math.cos(phi+s*offset),z.radius*math.sin(phi+s*offset))) for s in (-1,1)]

def common_tangencies(a,b):
    v=sub(b.center,a.center);d=norm(v);u=mul(v,1/d);perp=(-u[1],u[0])
    for sign in (-1,1):
        cosine=(a.radius-sign*b.radius)/d
        if abs(cosine)>1:continue
        sine=math.sqrt(max(0.,1-cosine*cosine))
        for side in (-1,1):
            n=add(mul(u,cosine),mul(perp,side*sine))
            yield add(a.center,mul(n,a.radius)),add(b.center,mul(n,sign*b.radius))

def shortest_route(start,goal,zones,max_zones=10):
    if not all(math.isfinite(v) for p in (start,goal) for v in p):raise ValueError('invalid_target')
    zones=merge_zones(zones)
    if len(zones)>max_zones:raise ValueError('too_many_zones')
    for point,name in ((start,'start_inside_zone'),(goal,'target_inside_zone')):
        if any(dist(point,z.center)<=z.radius+EPS for z in zones):raise ValueError(name)
    if dist(start,goal)<EPS:return Route((),0.,zones)
    nodes=[start,goal];graph=[[],[]];contacts=[[] for _ in zones]
    def contact(point,zone):
        # Dedup contacts on the same disc, including symmetric tangents.
        for idx in contacts[zone]:
            if dist(nodes[idx],point)<EPS:return idx
        idx=len(nodes);nodes.append(point);graph.append([]);contacts[zone].append(idx);return idx
    def connect(i,j):
        if segment_clear(nodes[i],nodes[j],zones):
            length=dist(nodes[i],nodes[j])
            graph[i].append((j,Leg(nodes[i],nodes[j],length)))
            graph[j].append((i,Leg(nodes[j],nodes[i],length)))
    connect(0,1)
    for k,z in enumerate(zones):
        for idx in (0,1):
            for point in point_tangencies(nodes[idx],z):connect(idx,contact(point,k))
    for i,a in enumerate(zones):
        for j in range(i+1,len(zones)):
            for pa,pb in common_tangencies(a,zones[j]):connect(contact(pa,i),contact(pb,j))
    for k,z in enumerate(zones):
        ordered=sorted(contacts[k],key=lambda idx: math.atan2(nodes[idx][1]-z.center[1],nodes[idx][0]-z.center[0]))
        for n,i in enumerate(ordered):
            if len(ordered)<2:continue
            j=ordered[(n+1)%len(ordered)]
            ai=math.atan2(nodes[i][1]-z.center[1],nodes[i][0]-z.center[0])
            aj=math.atan2(nodes[j][1]-z.center[1],nodes[j][0]-z.center[0])
            sweep=(aj-ai)%TAU
            graph[i].append((j,Leg(nodes[i],nodes[j],z.radius*sweep,z.center,z.radius,sweep)))
            graph[j].append((i,Leg(nodes[j],nodes[i],z.radius*sweep,z.center,z.radius,-sweep)))
    costs={0:0.};prev={};queue=[(0.,0)]
    while queue:
        cost,i=heapq.heappop(queue)
        if cost>costs[i]+EPS:continue
        if i==1:break
        for j,leg in graph[i]:
            candidate=cost+leg.length
            if candidate<costs.get(j,math.inf)-EPS:
                costs[j]=candidate;prev[j]=(i,leg);heapq.heappush(queue,(candidate,j))
    if 1 not in costs:raise ValueError('no_observed_route')
    legs=[];idx=1
    while idx:
        idx,leg=prev[idx];legs.append(leg)
    return Route(tuple(reversed(legs)),costs[1],zones)

def clusters_from_scan(scan,yaw_deg=0.,translation=(0.,0.),max_range=8.):
    """Invalid/no-return beams break clusters and remain unknown, never free."""
    values=scan['ranges'];step=scan['angle_increment'];amin=scan['angle_min']
    if not values or not math.isfinite(step) or not 0<abs(step)<=math.radians(2):raise ValueError('invalid_scan_geometry')
    if scan.get('truncated') is not False:raise ValueError('truncated_scan')
    if abs((amin+(len(values)-1)*step)-scan['angle_max'])>abs(step)*.1:raise ValueError('invalid_scan_geometry')
    yaw=math.radians(yaw_deg);groups=[];group=[];unknown=0
    for i,r in enumerate(values):
        valid=type(r) in (int,float) and math.isfinite(r) and scan['range_min']<=r<=scan['range_max']
        if not valid or r>max_range:
            if group:groups.append(group);group=[]
            if not valid:unknown+=1
            continue
        angle=amin+i*step+yaw;p=add(translation,(r*math.cos(angle),r*math.sin(angle)))
        if group and dist(group[-1],p)>.12+2.5*r*abs(step):groups.append(group);group=[]
        group.append(p)
    if group:groups.append(group)
    # Join wraparound only if endpoint beams exist and the scan covers 360 degrees.
    if len(groups)>1 and abs(len(values)*step)>=TAU-abs(step)*2:
        first,last=values[0],values[-1]
        if all(type(r) in (int,float) and math.isfinite(r) and scan['range_min']<=r<=min(max_range,scan['range_max']) for r in (first,last)):
            if dist(groups[-1][-1],groups[0][0])<=.12+2.5*max(first,last)*abs(step):
                groups=[groups[-1]+groups[0]]+groups[1:-1]
    return groups,unknown

class PathFollower:
    """Geometric lookahead commands only; not calibrated wheel-speed control.

    Returns STOP if the proposed short rollout crosses any inflated disc.
    Nominal speed/turn rate must be measured before motor integration.
    """
    def __init__(self,nominal_speed_mps,nominal_yaw_rps,lookahead=.45):
        if not .01<=nominal_speed_mps<=2 or not .01<=nominal_yaw_rps<=3 or not .1<=lookahead<=1:
            raise ValueError('invalid_motion_calibration')
        self.speed=nominal_speed_mps;self.yaw=nominal_yaw_rps;self.lookahead=lookahead

    def command(self,route):
        if not route.legs:return 0.,0.,'at_target'
        target=route.at(min(self.lookahead,route.length))
        heading=math.atan2(target[1],target[0]);turn=max(-.6,min(.6,1.5*heading))
        forward=0. if abs(heading)>=math.radians(23) else .30*max(.3,math.cos(heading))
        # Bound Cartesian rollout steps to <=1cm and inflate each swept segment
        # by a further 1cm to cover integration/chord error conservatively.
        padded=[Disc(z.center,z.radius+.01) for z in route.zones]
        p=(0.,0.);a=0.;dt=.01
        for _ in range(60):
            da=turn*self.yaw*dt;length=forward*self.speed*dt
            q=add(p,(length*math.cos(a+da*.5),length*math.sin(a+da*.5)))
            if not segment_clear(p,q,padded):return 0.,0.,'rollout_blocked'
            p=q;a+=da
        return forward,turn,'route_following'
