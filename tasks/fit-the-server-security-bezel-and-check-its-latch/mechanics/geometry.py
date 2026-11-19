"""Convex extrusion and bored seat from the declared fabrication dimensions."""
import math,numpy as np
import parts
def solid(vertices,color):
    p=np.array(vertices,float);lo=p.min(0);hi=p.max(0);at=(lo+hi)/2;n=len(p)//2
    faces=[tuple(range(n-1,-1,-1)),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
    return parts.Shape('mesh',tuple(hi-lo),tuple(at),color,vertices=tuple(map(tuple,p-at)),faces=tuple(faces))
def extrusion(profile,z0,z1,color):return solid([(x,y,z) for z in (z0,z1) for x,y in profile],color)
def ring_y(outer,inner,y0,y1,color,n=24,x=.003,z=0):
    out=[]
    for i in range(n):
        a,b=2*math.pi*i/n,2*math.pi*(i+1)/n
        out.append(solid([(x+r*math.cos(t),y,z+r*math.sin(t)) for y in (y0,y1) for r,t in ((outer,a),(outer,b),(inner,b),(inner,a))],color))
    return out
