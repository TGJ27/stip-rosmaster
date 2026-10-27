"""Single evidence snapshot per ROS /fire_inspection/confirmed rising edge."""
import json
import re
import time
import uuid
from pathlib import Path
SAFE=re.compile(r'^fire_[0-9]{8}_[0-9]{6}_[0-9a-f]{8}$')

def save_event(root,metadata,rgb=None,thermal=None):
    root=Path(root).expanduser();root.mkdir(parents=True,exist_ok=True)
    name='fire_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
    folder=root/name;folder.mkdir(mode=0o700)
    summary=dict(metadata)
    summary['event_id']=name
    summary['timestamp']=time.strftime('%Y-%m-%dT%H:%M:%S%z')
    summary['images']={}
    for label,data in (('rgb',rgb),('thermal',thermal)):
        if data and data.startswith(b'\xff\xd8') and len(data)<16*1024*1024:
            (folder/(label+'.jpg')).write_bytes(data)
            summary['images'][label]=True
    (folder/'event.json').write_text(json.dumps(summary,indent=2,default=str))
    return name

def list_events(root,limit=80):
    root=Path(root).expanduser()
    if not root.exists():return []
    entries=[]
    for p in root.glob('*/event.json'):
        if not SAFE.fullmatch(p.parent.name) or p.is_symlink() or p.parent.is_symlink():continue
        try:
            if p.stat().st_size>256*1024:continue
            value=json.loads(p.read_text())
            if isinstance(value,dict):entries.append(value)
        except (OSError,ValueError):continue
    return sorted(entries,key=lambda x:x.get('event_id',''),reverse=True)[:limit]

def image(root,event,kind):
    if not SAFE.fullmatch(event) or kind not in ('rgb','thermal'):
        raise ValueError('Invalid fire evidence request')
    root=Path(root).expanduser().resolve()
    folder=(root/event).resolve()
    if folder.parent!=root or not folder.is_dir():raise FileNotFoundError('Fire event not found')
    p=folder/(kind+'.jpg')
    if p.is_symlink() or not p.is_file() or p.stat().st_size>16*1024*1024:
        raise FileNotFoundError('Image unavailable')
    data=p.read_bytes()
    if not data.startswith(b'\xff\xd8'):raise ValueError('Invalid JPEG')
    return data
