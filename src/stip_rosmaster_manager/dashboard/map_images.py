"""Pure map preview and metadata helpers."""
import re
import struct
import zlib
from pathlib import Path

def pgm_to_png(path, max_size=450):
    """PGM P5 -> grayscale PNG, orientation matching ROS map viewer."""
    data=Path(path).read_bytes()
    i=0;words=[]
    while len(words)<4:
        while i<len(data) and data[i] in b' \r\n\t':i+=1
        if i<len(data) and data[i]==35:
            while i<len(data) and data[i]!=10:i+=1
            continue
        start=i
        while i<len(data) and data[i] not in b' \r\n\t':i+=1
        words.append(data[start:i])
    if words[0]!=b'P5':raise ValueError('Not binary PGM')
    w,h,scale=map(int,words[1:])
    if not(0<w<=20000 and 0<h<=20000 and w*h<=12000000 and scale<=255):
        raise ValueError('Unsupported PGM')
    if i>=len(data) or data[i] not in b' \r\n\t':raise ValueError('Invalid PGM header')
    i+=1
    rows=data[i:i+w*h]
    if len(rows)!=w*h:raise ValueError('PGM truncated')
    stride=max(1, int(max(w,h)/max_size))
    ww=(w+stride-1)//stride;hh=(h+stride-1)//stride
    out=bytearray()
    # map_saver already writes PGM top-down; do not invert it again.
    for y in range(0,h,stride):
        out.append(0)
        for v in rows[y*w:(y+1)*w:stride]:
            # map_saver uses 205 for unknown. Show the underlying map grid.
            out.extend((v,v,v,0 if v==205 else 255))
    def chunk(tag,p):return struct.pack('!I',len(p))+tag+p+struct.pack('!I',zlib.crc32(tag+p)&0xffffffff)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!IIBBBBB',ww,hh,8,6,0,0,0))+chunk(b'IDAT',zlib.compress(bytes(out),3))+chunk(b'IEND',b'')


def pgm_dimensions(path):
    with open(path,'rb') as f:
        tokens=[]
        while len(tokens)<4:
            line=f.readline()
            if not line:break
            line=line.split(b'#',1)[0]
            tokens.extend(line.split())
        if len(tokens)<4 or tokens[0]!=b'P5':raise ValueError('PGM header invalid')
        return int(tokens[1]),int(tokens[2])

def map_meta(path):
    data=Path(path).read_text()
    resolution=re.search(r'(?m)^resolution:\s*([^\s]+)',data)
    origin=re.search(r'(?m)^origin:\s*\[([^\]]+)\]',data)
    if not resolution or not origin:return None
    w,h=pgm_dimensions(str(path).replace('.yaml','.pgm'))
    return {'width':w,'height':h,'resolution':float(resolution.group(1)),
            'origin':[float(x) for x in origin.group(1).split(',')[:2]],
            'origin_yaw':float(origin.group(1).split(',')[2]),'saved':True}

