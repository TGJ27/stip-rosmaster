"""Atomic editing of existing STIP fire-inspection camera-arm timing parameters.

This module does not control servo motors. The ROS controller reads these
parameters at startup; only a controlled node restart applies changes.
"""
import os
from pathlib import Path
import re
import shutil
import tempfile
from datetime import datetime

FIELDS = ('max_servo_step_deg', 'min_servo_interval_sec')
PATTERN = re.compile(r'(?m)^(?P<space>[ \t]*)(?P<key>max_servo_step_deg|min_servo_interval_sec):[ \t]*(?P<value>[0-9]+(?:\.[0-9]+)?)(?P<suffix>[ \t]*(?:\#[^\n]*)?)$')

def read_tracking(path):
    text = Path(path).read_text()
    values = {match.group('key'): float(match.group('value')) for match in PATTERN.finditer(text)}
    if set(values) != set(FIELDS):
        raise ValueError('Fire inspection config is missing tracking speed parameters')
    return values

def save_tracking(path, step, interval):
    step, interval = float(step), float(interval)
    if not (0.5 <= step <= 3.0 and 0.18 <= interval <= 1.0):
        raise ValueError('Tracking step must be 0.5–3.0 degrees and interval 0.18–1.0 seconds')
    path=Path(path)
    source=path.read_text()
    previous=read_tracking(path)
    if abs(previous['max_servo_step_deg']-step)<1e-6 and abs(previous['min_servo_interval_sec']-interval)<1e-6:
        return {'changed':False,'values':previous}
    changes={'max_servo_step_deg':step,'min_servo_interval_sec':interval}
    def replace(match):
        return match.group('space')+match.group('key')+': '+format(changes[match.group('key')],'.2f')+match.group('suffix')
    updated,count=PATTERN.subn(replace,source)
    if count!=2:
        raise ValueError('Unexpected tracking config structure; no changes made')
    backup=path.with_name(path.name+'.backup_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    shutil.copy2(str(path),str(backup))
    descriptor,name=tempfile.mkstemp(prefix='.tracking_',dir=str(path.parent))
    try:
        with os.fdopen(descriptor,'w') as f:
            f.write(updated)
            f.flush();os.fsync(f.fileno())
        os.chmod(name,path.stat().st_mode & 0o777)
        os.replace(name,str(path))
    finally:
        if os.path.exists(name):os.unlink(name)
    return {'changed':True,'values':read_tracking(path),'backup':str(backup)}
