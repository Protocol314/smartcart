"""Mixed circle/wall local planning and bounded overlap recovery. No motor output."""
from dataclasses import dataclass
import heapq
import math
from zone_planner import EPS,TAU,Disc,Leg,add,sub,mul,dot,norm,dist,enclosing_zone

def closest(p,a,b):
    v=sub(b,a);vv=dot(v,v)
    return a if vv<EPS*EPS else add(a,mul(v,max(0.,min(1.,dot(sub(p,a),v)/vv))))

def cross(a,b):return a[0]*b[1]-a[1]*b[0]

def segment_distance(a,b,c,d):
    v,w=sub(b,a),sub(d,c);den=cross(v,w)
    if abs(den)>EPS:
        t=cross(sub(c,a),w)/den;u=cross(sub(c,a),v)/den
        if -EPS<=t<=1+EPS and -EPS<=u<=1+EPS:return 0.
    return min(dist(a,closest(a,c,d)),dist(b,closest(b,c,d)),
               dist(c,closest(c,a,b)),dist(d,closest(d,a,b)))

def hull(points):
    points=sorted(set(points))
    def half(items):
        result=[]
        for p in items:
            while len(result)>=2 and cross(sub(result[-1],result[-2]),sub(p,result[-1]))<=EPS:result.pop()
            result.append(p)
        return result
    return tuple(half(points)[:-1]+half(reversed(points))[:-1])

@dataclass(frozen=True)
class Obstacle:
    kind:str
    a:tuple
    b:tuple
    radius:float
    observed_length:float

    def clearance(self,point):
        return dist(point,closest(point,self.a,self.b))-self.radius

    def blocks(self,a,b):
        return segment_distance(a,b,self.a,self.b)<self.radius-EPS

    def polygon(self,sides=24):
        # Circumscribed hull: chord edges stay OUTSIDE the analytic capsule.
        r=(self.radius+.001)/math.cos(math.pi/sides)
        return hull([add(c,(r*math.cos(TAU*k/sides),r*math.sin(TAU*k/sides)))
                     for c in (self.a,self.b) for k in range(sides)])

def make_obstacles(groups,inflation,wall_threshold=.60,noise=.03):
    if not all(math.isfinite(x) for x in (inflation,wall_threshold,noise)) or inflation<0 or wall_threshold<=0 or noise<0:
        raise ValueError('invalid_zone_parameters')
    obstacles=[]
    for points in groups:
        if not points:continue
        if not all(len(p)==2 and all(math.isfinite(v) for v in p) for p in points):raise ValueError('invalid_cluster')
        a,b=points[0],points[-1];length=dist(a,b)
        if length+EPS>=wall_threshold:
            deviation=max(dist(p,closest(p,a,b)) for p in points)
            obstacles.append(Obstacle('wall',a,b,inflation+noise+deviation,length))
        else:
            disc=enclosing_zone(points,inflation,noise)
            obstacles.append(Obstacle('circle',disc.center,disc.center,disc.radius,length))
    return tuple(obstacles)

@dataclass(frozen=True)
class ScenePlan:
    state:str
    legs:tuple
    obstacles:tuple
    reason:str

    @property
    def length(self):return sum(l.length for l in self.legs)

    def as_dict(self):
        return {'state':self.state,'reason':self.reason,'length_m':self.length,
                'wall_threshold_m':.60,'motor_output':False,'drive_authorized':False,
                'obstacles':[{'kind':o.kind,'a':o.a,'b':o.b,'inflated_radius_m':o.radius,
                              'observed_length_m':o.observed_length} for o in self.obstacles],
                'legs':[{'kind':'line','start':l.start,'end':l.end,'length_m':l.length} for l in self.legs]}

def escape_route(start,goal,obstacles,step=.20):
    """One short translation proposal; not automatic permission to leave a zone.

    Every initially overlapping obstacle must have a nonnegative initial distance
    derivative. Distance to a convex capsule along a ray is convex, so this
    prevents entering deeper before exiting. Other obstacles remain forbidden.
    """
    initial=[o.clearance(start) for o in obstacles]
    colliding=[i for i,c in enumerate(initial) if c<=EPS]
    if not colliding:raise ValueError('escape_requires_overlap')
    best=None
    for k in range(180):
        angle=TAU*k/180;direction=(math.cos(angle),math.sin(angle))
        end=add(start,mul(direction,step));admissible=True
        for i,o in enumerate(obstacles):
            if i not in colliding:
                if o.blocks(start,end):admissible=False;break
            else:
                radial=sub(start,closest(start,o.a,o.b))
                if norm(radial)>EPS and dot(radial,direction)<-EPS:
                    admissible=False;break
        if not admissible:continue
        gains=[obstacles[i].clearance(end)-initial[i] for i in colliding]
        if min(gains)<-EPS or max(gains)<.005:continue
        # Sample too: catches numeric/degenerate capsule cases.
        previous=list(initial)
        for n in range(1,41):
            p=add(start,mul(direction,step*n/40))
            for i in colliding:
                value=obstacles[i].clearance(p)
                if value<previous[i]-EPS:admissible=False;break
                previous[i]=value
            if not admissible:break
        if not admissible:continue
        clearance=min(o.clearance(end) for o in obstacles)
        score=(clearance,min(gains),-dist(end,goal))
        if best is None or score>best[0]:best=(score,end)
    if best is None:return ScenePlan('STOP',(),obstacles,'no_nonworsening_escape')
    return ScenePlan('ESCAPE_PREVIEW',(Leg(start,best[1],step),),obstacles,
                     'replan_after_each_fresh_scan_no_motor_authorization')

def plan_scene(start,goal,obstacles,max_vertices=360):
    obstacles=tuple(obstacles)
    if not all(len(p)==2 and all(math.isfinite(v) for v in p) for p in (start,goal)):
        raise ValueError('invalid_target')
    if any(o.clearance(start)<=EPS for o in obstacles):return escape_route(start,goal,obstacles)
    if any(o.clearance(goal)<=EPS for o in obstacles):return ScenePlan('STOP',(),obstacles,'target_inside_zone')
    if dist(start,goal)<EPS:return ScenePlan('READY',(),obstacles,'at_target')
    def visible(a,b):return not any(o.blocks(a,b) for o in obstacles)
    if visible(start,goal):return ScenePlan('READY',(Leg(start,goal,dist(start,goal)),),obstacles,'direct_route')
    nodes=[start,goal]
    for o in obstacles:
        for p in o.polygon():
            if all(other.clearance(p)>=-EPS for other in obstacles):nodes.append(p)
    nodes=list(dict.fromkeys(nodes))
    if len(nodes)>max_vertices:return ScenePlan('STOP',(),obstacles,'scene_complexity_limit')
    # Candidate-vertex graph. No merging a long wall into an enormous circle.
    # Overlapping curved shapes may need extra contacts: failure returns STOP.
    queue=[(0.,0)];costs={0:0.};prev={};visited=set()
    while queue:
        cost,i=heapq.heappop(queue)
        if i in visited:continue
        visited.add(i)
        if i==1:break
        for j,p in enumerate(nodes):
            if j in visited:continue
            candidate=cost+dist(nodes[i],p)
            if candidate>=costs.get(j,math.inf)-EPS:continue
            if visible(nodes[i],p):
                costs[j]=candidate;prev[j]=i;heapq.heappush(queue,(candidate,j))
    if 1 not in visited:return ScenePlan('STOP',(),obstacles,'no_candidate_route')
    legs=[];idx=1
    while idx:
        before=prev[idx];legs.append(Leg(nodes[before],nodes[idx],dist(nodes[before],nodes[idx])));idx=before
    return ScenePlan('READY',tuple(reversed(legs)),obstacles,'shortest_visible_candidate_route')
