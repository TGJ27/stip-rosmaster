"""Read-only geometry parsing for working ROS inspection/detection topics.
Does not run a second detector or touch hardware drivers. Python 3.8 compatible.
"""
import math

def number(value):
    try:
        out=float(value)
        return out if math.isfinite(out) else None
    except (TypeError, ValueError):return None

def parse_qr_geometry(data):
    if len(data)<20:return None
    values=[number(x) for x in data[:20]]
    if any(x is None for x in values):return None
    _,w,h,aligned=values[:4]
    if w<1 or h<1 or w>10000 or h>10000:return None
    def pts(start):return [[values[i],values[i+1]] for i in range(start,start+8,2)]
    return {'width':int(w),'height':int(h),'aligned':bool(aligned),'qr':pts(4),'indicator':pts(12)}

def parse_fire_detections(data):
    if len(data)<8:return None
    head=[number(x) for x in data[:8]]
    if any(x is None for x in head):return None
    _,_,w,h,active,confidence,_,count=head
    if w<1 or h<1 or w>10000 or h>10000 or count<0 or count>100:return None
    boxes=[]
    for i in range(min(int(count),30)):
        j=8+6*i
        if j+6>len(data):break
        coords=[number(x) for x in data[j:j+6]]
        if any(v is None for v in coords):continue
        x1,y1,x2,y2,conf,cls=coords
        if not (0<=x1<x2<=w and 0<=y1<y2<=h):continue
        boxes.append({'x1':x1,'y1':y1,'x2':x2,'y2':y2,'confidence':conf,'class_id':int(cls)})
    return {'width':int(w),'height':int(h),'active':bool(active),'confidence':confidence,'boxes':boxes}
