"""Constrained read-only access to profiles owned by stip_qr_inspection."""
import json
import re
from pathlib import Path
SAFE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')
IMAGES = {'reference':'reference.jpg','annotated':'reference_annotated.jpg'}

def profile_directory(base, name):
    if not SAFE.fullmatch(str(name)):
        raise ValueError('Invalid profile name')
    root=Path(base).expanduser().resolve()
    path=(root/name).resolve()
    if path.parent != root or not path.is_dir() or path.is_symlink():
        raise FileNotFoundError('QR profile not found')
    return path

def get_profile(base,name):
    folder=profile_directory(base,name)
    file=folder/'profile.json'
    if file.is_symlink() or not file.is_file() or file.stat().st_size>256*1024:
        raise FileNotFoundError('QR profile data unavailable')
    payload=json.loads(file.read_text())
    if not isinstance(payload,dict):raise ValueError('Invalid profile JSON')
    return {'name':name,'qr':payload.get('qr',{}),'indicator':payload.get('indicator',{}),
            'camera_reference':payload.get('camera_reference',{}),'created_at':payload.get('created_at',''),
            'images':{k:(folder/f).is_file() for k,f in IMAGES.items()}}

def profile_image(base,name,kind):
    if kind not in IMAGES:raise ValueError('Invalid image kind')
    path=profile_directory(base,name)/IMAGES[kind]
    if path.is_symlink() or not path.is_file() or path.stat().st_size>16*1024*1024:
        raise FileNotFoundError('QR profile image unavailable')
    data=path.read_bytes()
    if not data.startswith(b'\xff\xd8'):raise ValueError('Invalid JPEG')
    return data
