"""Continuous circular-robot corridor vs 2D OccupancyGrid pixels.

Pure geometry for *experimental* Nav2 admission; not a hardware
collision controller. A centre segment swept by a disk is a capsule:
any costmap cell rectangle intersecting that capsule must be safe.
"""
from __future__ import annotations
import math

def _point_box2(px,py,l,b,r,t):
    dx=max(l-px,0.,px-r)
    dy=max(b-py,0.,py-t)
    return dx*dx+dy*dy

def _point_segment2(px,py,ax,ay,bx,by):
    dx,dy=bx-ax,by-ay
    length2=dx*dx+dy*dy
    k=max(0.,min(1.,((px-ax)*dx+(py-ay)*dy)/length2)) if length2 else 0.
    return (px-ax-k*dx)**2+(py-ay-k*dy)**2

def _crosses_box(ax,ay,bx,by,l,b,r,t):
    # Liang-Barsky clipped segment, touching the rectangle counts.
    dx,dy=bx-ax,by-ay
    lo,hi=0.,1.
    for p,q in ((-dx,ax-l),(dx,r-ax),(-dy,ay-b),(dy,t-ay)):
        if p==0.:
            if q<0.:return False
            continue
        k=q/p
        if p<0.:
            if k>hi:return False
            lo=max(lo,k)
        else:
            if k<lo:return False
            hi=min(hi,k)
    return lo<=hi

def _segment_box2(ax,ay,bx,by,l,b,r,t):
    if _crosses_box(ax,ay,bx,by,l,b,r,t):
        return 0.
    d=min(_point_box2(ax,ay,l,b,r,t),_point_box2(bx,by,l,b,r,t))
    for px,py in ((l,b),(l,t),(r,b),(r,t)):
        d=min(d,_point_segment2(px,py,ax,ay,bx,by))
    return d

def swept_disk_clear(raster,world_a,world_b,radius_m,max_cost):
    """Reject unsafe UNKNOWN/high-cost cells touching the *whole* moving disk.

    Convert to local costmap coordinates so rotated map origins work.
    Scan only the segment's expanded pixel bounding box, not the whole map.
    """
    co,si=math.cos(raster.yaw),math.sin(raster.yaw)
    def local(p):
        dx,dy=p[0]-raster.origin_x,p[1]-raster.origin_y
        return co*dx+si*dy,-si*dx+co*dy
    ax,ay=local(world_a);bx,by=local(world_b)
    if any(not math.isfinite(x) for x in (ax,ay,bx,by,radius_m)):
        return False
    res=raster.resolution
    x0=math.floor((min(ax,bx)-radius_m)/res)
    x1=math.floor((max(ax,bx)+radius_m)/res)
    y0=math.floor((min(ay,by)-radius_m)/res)
    y1=math.floor((max(ay,by)+radius_m)/res)
    if x0<0 or y0<0 or x1>=raster.width or y1>=raster.height:
        return False # unobserved rolling costmap edge cannot be safe
    radius2=radius_m*radius_m
    for row in range(y0,y1+1):
        bottom=row*res
        for col in range(x0,x1+1):
            left=col*res
            if _segment_box2(ax,ay,bx,by,left,bottom,left+res,bottom+res)<=radius2+1e-12:
                cost=raster.data[row*raster.width+col]
                if cost<0 or cost>max_cost:return False
    return True
