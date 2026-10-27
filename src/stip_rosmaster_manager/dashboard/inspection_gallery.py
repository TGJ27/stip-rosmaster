"""Read-only, constrained gallery for the existing stip_qr_inspection run artifacts.

The inspection ROS package owns capture, analysis and persistence. This module
only indexes its existing result.json / annotated.jpg / indicator_crop.jpg.
"""
import json
import re
from pathlib import Path

SAFE = re.compile(r'^[A-Za-z0-9_-]{1,100}$')
FILES = {'annotated': 'annotated.jpg', 'original': 'original.jpg', 'indicator': 'indicator_crop.jpg'}


def root_path(base=None):
    return (Path(base).expanduser() if base is not None else
            Path.home() / 'Documents' / 'stip_inspection_data' / 'runs').resolve()


def _inside(path, root):
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def get_run_path(root, profile, run):
    if not SAFE.fullmatch(profile) or not SAFE.fullmatch(run):
        raise ValueError('Invalid inspection identifier')
    root = root_path(root)
    path = (root / profile / run).resolve()
    if not _inside(path, root) or not path.is_dir():
        raise FileNotFoundError('Inspection run not found')
    return path


def list_inspection_runs(base=None, limit=80):
    root = root_path(base)
    if not root.exists():
        return []
    entries = []
    for p in root.glob('*/*/result.json'):
        folder = p.parent
        if not SAFE.fullmatch(folder.name) or not SAFE.fullmatch(folder.parent.name):
            continue
        if not _inside(folder.resolve(), root) or p.is_symlink():
            continue
        try:
            payload = json.loads(p.read_text())
            stamp = p.stat().st_mtime
        except (OSError, ValueError, TypeError):
            continue
        if not isinstance(payload, dict):
            continue
        files = {kind: (folder / filename).is_file() and not (folder / filename).is_symlink()
                 for kind, filename in FILES.items()}
        entries.append({'profile': folder.parent.name, 'run': folder.name,
                        'timestamp': str(payload.get('timestamp', ''))[:40],
                        'inspection_name': str(payload.get('inspection_name', folder.parent.name))[:100],
                        'aligned': bool(payload.get('aligned', False)),
                        'message': str(payload.get('message', ''))[:220],
                        'indicator_status': str((payload.get('indicator_result') or {}).get('status', 'UNKNOWN'))[:45],
                        'images': files, '_mtime': stamp})
    entries.sort(key=lambda x: x['_mtime'], reverse=True)
    for item in entries[:limit]:
        item.pop('_mtime', None)
    return entries[:limit]


def load_inspection_image(base, profile, run, kind):
    if kind not in FILES:
        raise ValueError('Unknown inspection image type')
    folder = get_run_path(base, profile, run)
    path = folder / FILES[kind]
    if path.is_symlink() or not path.is_file():
        raise FileNotFoundError('Inspection image not found')
    if path.stat().st_size > 16 * 1024 * 1024:
        raise ValueError('Image too large')
    raw = path.read_bytes()
    if not raw.startswith(b'\xff\xd8'):
        raise ValueError('Invalid JPEG image')
    return raw
