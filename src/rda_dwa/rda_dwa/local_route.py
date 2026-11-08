"""Bounded static configuration-space guide with complete rectangle occupancy.
The guide is a heuristic; every executable trajectory still receives timed OBB
validation. Long road boundaries have no distant-corner escape shortcut.
"""
import math,heapq
import numpy as np
from .motion_safety import minimum_gaps


def route_waypoint(centre,yaw,goal,obs,geometry,margin=.12,resolution=.05,padding=.045):
 static=[o for o in obs if o.speed<=.1]
 if not static:return tuple(goal),True
 sx,sy=centre;gx,gy=goal
 xmin=min(sx,gx)-2.;xmax=max(sx,gx)+2.;ymin=min(sy,gy)-2.5;ymax=max(sy,gy)+2.5
 nx=int(math.ceil((xmax-xmin)/resolution))+1;ny=int(math.ceil((ymax-ymin)/resolution))+1
 if nx*ny>40000:return tuple(goal),False
 xs=xmin+np.arange(nx)*resolution;ys=ymin+np.arange(ny)*resolution
 xx,yy=np.meshgrid(xs,ys,indexing='ij');poses=np.c_[xx.ravel(),yy.ravel(),np.full(nx*ny,yaw)]
 gaps=minimum_gaps(poses[:,None,:],[0.],static,geometry)
 occupied=(gaps<margin+padding).reshape(nx,ny)
 def index(x,y):return (max(0,min(nx-1,round((x-xmin)/resolution))),max(0,min(ny-1,round((y-ymin)/resolution))))
 start=index(sx,sy);end=index(gx,gy)
 if gaps.reshape(nx,ny)[start]<margin or occupied[end]:return tuple(goal),False
 occupied[start]=False
 def point(i):return np.array([xs[i[0]],ys[i[1]]])
 distance={start:0.};previous={};heap=[(math.dist(start,end),0.,start)];visited=set()
 moves=[(a,b) for a in (-1,0,1) for b in (-1,0,1) if a or b]
 while heap:
  _,cost,i=heapq.heappop(heap)
  if i in visited:continue
  visited.add(i)
  if i==end:break
  for a,b in moves:
   j=(i[0]+a,i[1]+b)
   if j[0]<0 or j[0]>=nx or j[1]<0 or j[1]>=ny or occupied[j]:continue
   if a and b and (occupied[(i[0]+a,i[1])] or occupied[(i[0],i[1]+b)]):continue
   candidate=cost+math.hypot(a,b)
   if candidate<distance.get(j,math.inf):
    distance[j]=candidate;previous[j]=i;heapq.heappush(heap,(candidate+math.dist(j,end),candidate,j))
 if end not in visited:
  if padding>.016:return route_waypoint(centre,yaw,goal,obs,geometry,margin,resolution,.015)
  return tuple(goal),False
 path=[end]
 while path[-1]!=start:path.append(previous[path[-1]])
 path.reverse();current=np.array([sx,sy]);chosen=point(path[min(2,len(path)-1)])
 # Furthest locally visible waypoint, bounded distance. Stops at the first
 # blocked shortcut rather than taking an unvalidated guide-box corner.
 for index_ in path[1:]:
  candidate=point(index_);length=math.dist(current,candidate)
  if length>3.5:break
  u=np.linspace(0,1,max(2,int(math.ceil(length/.04))+1))
  pts=current[None,:]+u[:,None]*(candidate-current)[None,:]
  pg=np.c_[pts,np.full(len(pts),yaw)]
  if float(minimum_gaps(pg,[0.]*len(pg),static,geometry))<margin+max(.005,padding-.01):break
  chosen=candidate
 if math.dist(chosen,goal)<.15:chosen=np.array(goal)
 return tuple(chosen),True
