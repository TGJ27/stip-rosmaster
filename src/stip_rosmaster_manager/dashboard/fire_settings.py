"""Configure the ACTUAL native C++ ROS detector startup thresholds.

fire_detector_cpp reads its private parameters at process startup; the
stage manager must restart the fire stage for changes to take effect.
"""
import os
import re
import shutil
import tempfile
from pathlib import Path
KEYS = ('fire_confidence_threshold','smoke_confidence_threshold')

def read_thresholds(path):
    content=Path(path).read_text()
    values={}
    for key in KEYS:
        m=re.search(r'(?m)^\s*'+key+r':\s*([0-9.]+)\s*(?:#.*)?$',content)
        if not m:raise ValueError('Missing ROS detector setting '+key)
        values[key]=float(m.group(1))
    return values

def write_thresholds(path,fire,smoke):
    fire=float(fire);smoke=float(smoke)
    if not (0.1<=fire<=0.95 and 0.1<=smoke<=0.95):
        raise ValueError('Confidence must be between 0.10 and 0.95')
    path=Path(path)
    original=path.read_text()
    read_thresholds(path)
    contents=original
    for key,value in zip(KEYS,(fire,smoke)):
        contents,count=re.subn(r'(?m)^(\s*'+key+r':\s*)[0-9.]+',
                               lambda m:m.group(1)+format(value,'.2f'),contents,count=1)
        if count!=1:raise ValueError('Could not update '+key)
    if contents==original:return False
    backup=path.with_suffix(path.suffix+'.dashboard.bak')
    if not backup.exists():shutil.copy2(str(path),str(backup))
    fd,temp=tempfile.mkstemp(prefix='.fire_settings_',dir=str(path.parent))
    try:
        os.fchmod(fd,path.stat().st_mode & 0o777)
        with os.fdopen(fd,'w') as f:f.write(contents)
        os.replace(temp,str(path))
    finally:
        if os.path.exists(temp):os.unlink(temp)
    return True
