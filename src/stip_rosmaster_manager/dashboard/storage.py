"""Mission repository: atomic JSON, safe identifiers, explicit map binding."""
import json
import os
import re
import tempfile
import time
from pathlib import Path

VALID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')


def safe_name(value):
    if not isinstance(value, str) or not VALID.fullmatch(value):
        raise ValueError('Name: use 1–64 ASCII letters, digits, underscore or hyphen')
    return value


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(prefix='.pending_', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, str(path))
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


class Repository:
    def __init__(self, maps_dir, data_dir):
        self.maps_dir = Path(maps_dir).expanduser().resolve()
        self.data_dir = Path(data_dir).expanduser().resolve()
        self.missions = self.data_dir / 'missions'
        self.missions.mkdir(parents=True, exist_ok=True)
        self.maps_dir.mkdir(parents=True, exist_ok=True)

    def maps(self):
        result = []
        for meta in sorted(self.maps_dir.glob('*.yaml')):
            if not VALID.fullmatch(meta.stem):
                continue
            if (self.maps_dir / (meta.stem + '.pgm')).exists():
                result.append({'name': meta.stem, 'modified': int(meta.stat().st_mtime),
                               'missions': sum(m.get('map') == meta.stem for m in self.list_missions())})
        return result

    def map_exists(self, name):
        safe_name(name)
        return (self.maps_dir / (name + '.yaml')).is_file() and (self.maps_dir / (name + '.pgm')).is_file()

    def delete_map(self, name):
        safe_name(name)
        if any(m.get('map') == name for m in self.list_missions()):
            raise ValueError('Map is referenced by a saved mission')
        image = self.maps_dir / (name + '.pgm')
        metadata = self.maps_dir / (name + '.yaml')
        if not image.is_file() or not metadata.is_file():
            raise FileNotFoundError('Saved map pair not found')
        # Only delete the two verified files inside the configured map directory.
        metadata.unlink()
        image.unlink()
        return {'deleted': True, 'name': name}

    def list_missions(self):
        result = []
        for file in sorted(self.missions.glob('*.json')):
            try:
                obj = json.loads(file.read_text())
                if isinstance(obj, dict) and obj.get('id') == file.stem:
                    result.append(obj)
            except (ValueError, OSError):
                continue
        return result

    def get(self, name):
        path = self.missions / (safe_name(name) + '.json')
        if not path.exists():
            raise FileNotFoundError('Mission not found')
        return json.loads(path.read_text())

    def save(self, body):
        mid = safe_name(body.get('id', ''))
        map_id = safe_name(body.get('map', ''))
        if not self.map_exists(map_id):
            raise ValueError('Associated map is missing')
        points = body.get('waypoints', [])
        if not isinstance(points, list) or len(points) > 200:
            raise ValueError('Waypoints must be a list with no more than 200 entries')
        seen = set()
        cleaned = []
        for i, point in enumerate(points):
            if not isinstance(point, dict):
                raise ValueError('Invalid waypoint')
            pid = safe_name(point.get('id') or 'P{}'.format(i + 1))
            if pid in seen:
                raise ValueError('Duplicate waypoint identifier')
            seen.add(pid)
            coords = [float(point.get(k, 0)) for k in ('x', 'y', 'yaw')]
            if not all(abs(v) < 100000 for v in coords):
                raise ValueError('Invalid waypoint position')
            tasks = []
            for task in point.get('tasks', []):
                kind = task.get('type')
                if kind == 'pause':
                    duration = float(task.get('seconds', 0))
                    if not 0 <= duration <= 3600:
                        raise ValueError('Pause must be 0–3600 seconds')
                    tasks.append({'type': 'pause', 'seconds': duration})
                elif kind == 'qr':
                    name = safe_name(task.get('profile', ''))
                    tasks.append({'type': 'qr', 'profile': name,
                                  'pan': float(task.get('pan', 90)), 'tilt': float(task.get('tilt', 0)),
                                  'qr_value': str(task.get('qr_value', ''))[:2048]})
                else:
                    raise ValueError('Only QR inspection and Pause tasks are supported')
            cleaned.append({'id': pid, 'name': str(point.get('name') or pid)[:90],
                            'x': coords[0], 'y': coords[1], 'yaw': coords[2], 'tasks': tasks})
        value = {'version': 1, 'id': mid, 'name': str(body.get('name') or mid)[:100],
                 'map': map_id, 'waypoints': cleaned, 'loop': bool(body.get('loop', False)),
                 'updated': int(time.time())}
        atomic_json(self.missions / (mid + '.json'), value)
        return value

    def delete_mission(self, name):
        path = self.missions / (safe_name(name) + '.json')
        path.unlink()
