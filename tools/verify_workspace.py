#!/usr/bin/env python3
"""Offline static verifier: code, ROS XML includes, and preserved artifacts."""
import ast
from pathlib import Path
import hashlib
import re
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
PKGS={p.name:p for p in (ROOT/'src').iterdir() if (p/'package.xml').is_file()}
errors=[]; warnings=[]; count={'py':0,'xml':0,'launch':0,'files':0}
for p in ROOT.rglob('*.py'):
    try: ast.parse(p.read_text(encoding='utf-8'),filename=str(p))
    except Exception as exc:errors.append('Python {}: {}'.format(p.relative_to(ROOT),exc))
    count['py']+=1
for p in ROOT.rglob('*'):
    if not p.is_file() or p.suffix not in {'.xml','.launch'}:continue
    try: tree=ET.parse(p)
    except Exception as exc:
        errors.append('XML {}: {}'.format(p.relative_to(ROOT),exc));continue
    count['xml']+=1
    if p.suffix=='.launch':
        count['launch']+=1
        for inc in tree.findall('.//include'):
            val=inc.attrib.get('file','')
            found=re.match(r'^\$\(find ([\w-]+)\)/(.*)$',val)
            if found:
                pkg,rest=found.groups()
                if pkg in PKGS and not (PKGS[pkg]/rest).is_file():
                    errors.append('Missing include: {} -> {}'.format(p.relative_to(ROOT),val))
                elif pkg not in PKGS:
                    warnings.append('External ROS include: {} (from {})'.format(pkg,p.relative_to(ROOT)))
manifest=ROOT/'MANIFEST_SHA256.txt'
if not manifest.exists():errors.append('Missing integrity manifest')
else:
    for line in manifest.read_text().splitlines():
        if not line or line.startswith('#'):continue
        digest, name=line.split('  ',1)
        f=ROOT/name
        if not f.is_file() or hashlib.sha256(f.read_bytes()).hexdigest()!=digest:
            errors.append('Checksum mismatch: {}'.format(name))
        count['files']+=1
if not (ROOT/'src/stip_fire_detection/models/fire_smoke_yolov8s.engine').is_file():
    errors.append('Missing TensorRT engine')
if not (ROOT/'src/stip_rosmaster_nav/maps/my_map.pgm').is_file():
    errors.append('Missing original map image')
if not (ROOT/'src/CMakeLists.txt').is_symlink():
    warnings.append('src/CMakeLists.txt is not a symlink; ./build_local.sh can repair a flattened link')
print('Workspace:',ROOT)
print('Packages:',len(PKGS),'Launches:',count['launch'],'Python:',count['py'],'XML:',count['xml'],'Manifest entries:',count['files'])
for s in sorted(set(warnings)):print('WARNING:',s)
for s in errors:print('ERROR:',s)
print('STATIC RESULT: '+('PASS' if not errors else 'FAIL: {} error(s)'.format(len(errors))))
sys.exit(1 if errors else 0)
