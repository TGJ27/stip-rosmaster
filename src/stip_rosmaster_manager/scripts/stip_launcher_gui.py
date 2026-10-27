#!/usr/bin/env python3
"""STIP ROSMASTER launcher: high-level workflows, calibration and component tests.

Python 3.8 + stock Tk. Main System stage recovery remains solely with the
existing STIP stage manager. Only one managed roslaunch at a time, and refuse
to start when another ROS graph already contains nodes (shared device safety).
"""
import collections
import datetime
import json
import math
import os
from pathlib import Path
import queue
import re
import tempfile
import secrets
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox

WORKSPACE = Path.home() / 'Documents' / 'stip_rosmaster_ws'
MANAGER_PKG = WORKSPACE / 'src' / 'stip_rosmaster_manager'
WATCHER = MANAGER_PKG / 'scripts' / 'stip_stage_watcher.py'
LOG_DIR = Path.home() / '.ros' / 'stip_launcher_logs'
MAP_DIR = WORKSPACE / 'src' / 'stip_rosmaster_nav' / 'maps'
MAP_NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')

WORKFLOWS = [
    ('system', 'Main System', 'Staged startup and individual recovery',
     'stip_rosmaster_manager', 'system.launch', 'System Manager'),
    ('fire', 'Fire Inspection', 'AI, thermal verification, tracking, and alarm',
     'stip_fire_inspection', 'fire_inspection_live.launch', 'Fire Inspection'),
    ('qr', 'QR Inspection', 'Standalone QR inspection and camera tracking',
     'stip_qr_inspection', 'inspection.launch', 'QR Inspection'),
    ('mapping', 'SLAM Mapping', 'Bringup, LiDAR and GMapping',
     'stip_rosmaster_nav', 'mapping_full.launch', 'SLAM Mapping'),
    ('navigation', 'Autonomous Navigation', 'Bringup, LiDAR, AMCL and move_base',
     'stip_rosmaster_nav', 'navigation_full.launch', 'Navigation'),
]
WORKFLOW = {w[0]: w for w in WORKFLOWS}
# All tools invoke launch files ALREADY PROVIDED by the existing workspace.
# Fields: key, label, description, ROS package, launch filename, window type.
# Each test is standalone: it starts its dependencies only via its original XML.
TOOL_GROUPS = {
    'calibration': [
        ('thermal_align', 'RGB / Thermal Alignment',
         'Manual alignment GUI (WASD, scale, rotation and save). Opens two cameras.',
         'stip_thermal_camera', 'thermal_rgb_calibrate.launch', 'Calibration GUI'),
        ('gas_baseline', 'MQ-6 Baseline',
         'POP32i serial + clean-air MQ-6 baseline calibration window.',
         'stip_gas_sensor', 'mq6_calibrate.launch', 'Calibration GUI'),
        ('linear_calibration', 'Linear X / Y (rqt)',
          'Yahboom linear odometry calibration via rqt_reconfigure; STIP bringup included.',
          'stip_rosmaster_manager', 'calibrate_linear.launch', 'rqt_reconfigure'),
        ('angular_calibration', 'Angular (rqt)',
          'Yahboom rotation odometry calibration via rqt_reconfigure; STIP bringup included.',
          'stip_rosmaster_manager', 'calibrate_angular.launch', 'rqt_reconfigure'),
        ('imu_calibration', 'IMU Bias Capture',
          'Yahboom IMU bias capture using /imu/data_raw; robot must stay stationary.',
          'stip_rosmaster_manager', 'calibrate_imu.launch', 'ROS calibration node'),
    ],
    'tests': [
        ('rgb_test', 'RGB Camera',
         'USB RGB publisher with a separate live preview window.',
         'stip_inspection_camera', 'camera_test.launch', 'Live preview'),
        ('thermal_test', 'Thermal Camera',
         'MLX90640 temperature viewer with INFERNO color map.',
         'stip_thermal_camera', 'thermal_test.launch', 'Live preview'),
        ('rgb_thermal_test', 'RGB + Thermal',
         'Standalone aligned RGB and thermal overlay window.',
         'stip_thermal_camera', 'thermal_rgb_view.launch', 'Live overlay'),
        ('arm_test', 'Camera Arm',
         'Dedicated ROS camera-arm GUI with live servo publisher; starts bringup.',
         'stip_camera_arm', 'teleop_gui.launch', 'ROS GUI window'),
        ('cpp_fire_test', 'C++ Fire Detection',
         'TensorRT fire/smoke detector with RGB bounding-box viewer.',
         'stip_fire_detection', 'fire_detection_cpp.launch', 'AI preview'),
        ('gas_test', 'MQ-6 / POP32i',
         'Serial gas bridge only. Inspect messages and status in logs / ROS.',
         'stip_gas_sensor', 'gas_sensor.launch', 'ROS only · no GUI'),
        ('lidar_test', 'LiDAR',
         'LiDAR ROS publisher only; inspect scan topics in ROS tools.',
         'stip_rosmaster_nav', 'lidar.launch', 'ROS only · no GUI'),
        ('base_test', 'Robot Bringup',
         'Robot driver, odometry, IMU and EKF, without inspection modules.',
         'stip_rosmaster_bringup', 'bringup.launch', 'ROS only · no GUI'),
        ('model_test', 'Robot Model',
         'URDF and joint-state visualization in RViz.',
         'stip_rosmaster_description', 'view_model.launch', 'RViz GUI'),
    ],
}
TOOLS = {entry[0]: entry for group in TOOL_GROUPS.values() for entry in group}
CATALOG = dict(WORKFLOW)
CATALOG.update(TOOLS)
STAGE_NAMES = [
    ('ui', 'Inspection UI'), ('bringup', 'Robot Bringup'),
    ('camera', 'RGB Camera'), ('qr', 'QR Detection'),
    ('fire', 'Fire Detector C++'), ('gas', 'MQ-6 / POP32i'),
    ('thermal', 'MLX90640 Thermal'),
]
COLOR = {
    'bg': '#0e1522', 'panel': '#172130', 'surface': '#202e40',
    'text': '#f1f5f9', 'muted': '#95a5ba', 'green': '#49d4a7',
    'yellow': '#f1be72', 'red': '#f28791', 'accent': '#70b2ff',
    'border': '#344558', 'quiet': '#121d2b',
}



def round_polygon(x1, y1, x2, y2, radius):
    """Sampled quarter circles, so this works with stock Tk 8.6."""
    r = max(0.0, min(float(radius), (x2-x1)/2.0, (y2-y1)/2.0))
    points = []
    for cx, cy, a0 in ((x2-r, y1+r, -90), (x2-r, y2-r, 0),
                       (x1+r, y2-r, 90), (x1+r, y1+r, 180)):
        for step in range(7):
            angle = math.radians(a0 + step * 90 / 6.0)
            points.extend((cx + r*math.cos(angle), cy + r*math.sin(angle)))
    return points


class RoundedPanel(tk.Canvas):
    """Actual rounded canvas surface with ordinary Tk widgets in its interior."""
    def __init__(self, parent, height=100, radius=18, padding=14, fill=None):
        self.surface = fill or COLOR['panel']
        self.radius = radius
        self.inset = padding
        super().__init__(parent, height=height, highlightthickness=0, bd=0,
                         bg=parent.cget('bg'))
        self.body = tk.Frame(self, bg=self.surface)
        self.embedded = self.create_window(padding, padding, anchor='nw', window=self.body)
        self.bind('<Configure>', self._draw)

    def _draw(self, event=None):
        w, h = self.winfo_width(), self.winfo_height()
        if w < 4 or h < 4:
            return
        self.delete('panel_shape')
        self.create_polygon(round_polygon(0, 0, w-1, h-1, self.radius),
                            fill=self.surface, outline='', tags='panel_shape')
        self.tag_lower('panel_shape')
        self.coords(self.embedded, self.inset, self.inset)
        self.itemconfigure(self.embedded, width=max(1, w-2*self.inset),
                           height=max(1, h-2*self.inset))


class RoundedButton(tk.Canvas):
    """Keyboard-accessible, hover-aware button without external dependencies."""
    def __init__(self, parent, text, command, width=130, height=39,
                 kind='ghost', align='center', radius=12, font_size=10):
        super().__init__(parent, width=width, height=height, highlightthickness=0,
                         bd=0, bg=parent.cget('bg'), cursor='hand2',
                         takefocus=1)
        self.text = text
        self.command = command
        self.kind = kind
        self.text_align = align
        self.radius = radius
        self.font_size = font_size
        self.enabled = True
        self.selected = False
        self.hover = False
        self.bind('<Configure>', self._draw)
        self.bind('<Enter>', self._enter)
        self.bind('<Leave>', self._leave)
        self.bind('<Button-1>', self._click)
        self.bind('<Return>', self._click)
        self.bind('<space>', self._click)
        self.bind('<FocusIn>', self._draw)
        self.bind('<FocusOut>', self._draw)

    def _enter(self, _event):
        self.hover = True
        self._draw()

    def _leave(self, _event):
        self.hover = False
        self._draw()

    def _click(self, _event):
        if self.enabled:
            self.command()

    def set_enabled(self, value):
        self.enabled = bool(value)
        self.configure(cursor='hand2' if self.enabled else 'arrow')
        self._draw()

    def set_selected(self, value):
        self.selected = bool(value)
        self._draw()

    def set_text(self, value):
        self.text = value
        self._draw()

    def _draw(self, _event=None):
        self.delete('all')
        w, h = self.winfo_width(), self.winfo_height()
        if w < 5 or h < 5:
            w, h = int(self.cget('width')), int(self.cget('height'))
        if self.kind == 'primary':
            base, txt = COLOR['green'], COLOR['bg']
        elif self.kind == 'danger':
            base, txt = COLOR['surface'], COLOR['red']
        elif self.kind == 'nav':
            base = COLOR['surface'] if self.selected else COLOR['panel']
            txt = COLOR['text'] if self.selected else COLOR['muted']
        else:
            base, txt = COLOR['surface'], COLOR['text']
        if self.hover and self.enabled and self.kind != 'primary':
            base = COLOR['border']
        if not self.enabled:
            base = COLOR['quiet']
            txt = COLOR['muted']
        if self.selected and self.kind == 'nav':
            txt = COLOR['accent']
        self.create_polygon(round_polygon(1, 1, w-2, h-2, self.radius),
                            fill=base, outline='')
        if self.selected and self.kind == 'nav':
            self.create_oval(12, h/2-3, 18, h/2+3, fill=COLOR['accent'], outline='')
        x = 30 if self.text_align == 'left' else w/2
        anchor = 'w' if self.text_align == 'left' else 'center'
        self.create_text(x, h/2, text=self.text, fill=txt,
                         font=('DejaVu Sans', self.font_size, 'bold'), anchor=anchor)
        # Tk focus_get() resolves widget names through _nametowidget; on Tk 8.6
        # that can raise KeyError('__tk__messagebox') after modal dialogs.
        # Raw Tcl focus path does not construct a widget object.
        try:
            focused = str(self.tk.call('focus')) == str(self)
        except tk.TclError:
            focused = False
        if focused and self.enabled:
            self.create_line(12, h-3, w-12, h-3, fill=COLOR['accent'])


class StageRow(RoundedPanel):
    def __init__(self, parent, key, title, callback):
        super().__init__(parent, height=48, radius=11, padding=6, fill=COLOR['surface'])
        self.key = key
        # Pack right controls FIRST to reserve enough width on smaller monitors.
        self.button = RoundedButton(self.body, text='Retry', command=lambda: callback(key),
                                    width=54, height=27, kind='ghost', radius=9, font_size=8)
        self.button.pack(side='right', padx=(7,0))
        self.button.set_enabled(False)
        self.status = tk.Label(self.body, text='WAITING', bg=self.surface, fg=COLOR['muted'],
                               width=10, anchor='e', font=('DejaVu Sans', 8, 'bold'))
        self.status.pack(side='right')
        left = tk.Frame(self.body, bg=self.surface)
        left.pack(side='left', fill='both', expand=True)
        tk.Label(left, text=title, bg=self.surface, fg=COLOR['text'],
                 font=('DejaVu Sans', 9, 'bold'), anchor='w').pack(fill='x')
        self.detail = tk.Label(left, text='Not started', bg=self.surface, fg=COLOR['muted'],
                               font=('DejaVu Sans', 8), anchor='w')
        self.detail.pack(fill='x', pady=(1,0))


def ros_environment():
    """Read ROS environment once; works even when launched from Ubuntu menu."""
    setup = [Path('/opt/ros/noetic/setup.bash'), WORKSPACE / 'devel' / 'setup.bash']
    missing = [str(p) for p in setup if not p.exists()]
    if missing:
        raise RuntimeError('Missing ROS setup: ' + ', '.join(missing))
    snippet = 'set -e; ' + '; '.join('source ' + shlex.quote(str(p)) for p in setup) + '; env -0'
    result = subprocess.run(['/bin/bash', '-c', snippet], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=15, check=True)
    env = dict(os.environ)
    for chunk in result.stdout.split(b'\0'):
        if b'=' in chunk:
            k, value = chunk.split(b'=', 1)
            if k:
                env[os.fsdecode(k)] = os.fsdecode(value)
    env['PYTHONUNBUFFERED'] = '1'
    return env


# Every workflow receives a random environment marker. Children of roslaunch
# inherit it even when they create new process groups or their parent exits.
# We can therefore clean up ONLY our own nodes, never other users' ROS nodes.
PROCESS_MARKER = 'STIP_ROSMASTER_LAUNCH_TOKEN'


def list_usable_maps():
    """Navigation XML loads $(find stip_rosmaster_nav)/maps/<name>.yaml.

    Map YAML is considered usable only if its referenced image exists.
    """
    if not MAP_DIR.is_dir():
        return []
    names = []
    for yaml_file in MAP_DIR.glob('*.yaml'):
        if not MAP_NAME_RE.fullmatch(yaml_file.stem):
            continue
        try:
            data = yaml_file.read_text(encoding='utf-8')
            match = re.search(r'^\s*image:\s*(.+?)\s*$', data, re.MULTILINE)
            if not match:
                continue
            image_name = match.group(1).strip().strip('"\'')
            image_file = Path(image_name)
            if not image_file.is_absolute():
                image_file = yaml_file.parent / image_file
            if not image_file.is_file():
                continue
            names.append(yaml_file.stem)
        except (OSError, UnicodeError, ValueError):
            continue
    return sorted(names, key=str.casefold)



def tagged_processes(token):
    """Return live PID/command pairs carrying our exact launch token (Linux)."""
    if not token:
        return []
    marker = (PROCESS_MARKER + '=' + token).encode('ascii')
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == os.getpid():
            continue
        try:
            env = (entry / 'environ').read_bytes().split(b'\0')
            if marker not in env:
                continue
            cmd = (entry / 'comm').read_text(errors='replace').strip()
            found.append((pid, cmd))
        except (FileNotFoundError, ProcessLookupError, PermissionError, OSError):
            # Process exited mid-scan, or belongs to another user.
            continue
    return found


def other_tagged_processes():
    """Find abandoned or concurrently running STIP-managed workflows."""
    found = []
    prefix = (PROCESS_MARKER + '=').encode('ascii')
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            pid = int(entry.name)
            if pid == os.getpid():
                continue
            if any(field.startswith(prefix) for field in (entry / 'environ').read_bytes().split(b'\0')):
                found.append(pid)
        except (FileNotFoundError, ProcessLookupError, PermissionError, OSError):
            continue
    return found


def wait_until_gone(token, seconds):
    deadline = time.monotonic() + seconds
    while True:
        active = tagged_processes(token)
        if not active or time.monotonic() >= deadline:
            return active
        time.sleep(0.25)


def signal_tagged(token, sig):
    """Signal only processes whose environment still matches our token."""
    for pid, _ in tagged_processes(token):
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass


def stop_roslaunch(proc, token):
    """Graceful roslaunch shutdown, then bounded cleanup of orphaned children."""
    # roslaunch normally relays SIGINT to its own registered nodes.
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            try:
                proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                if proc.poll() is None:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    try:
                        proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        pass
    # IMPORTANT: parent exit is NOT sufficient evidence that its child nodes
    # or GUI processes exited. They may have created their own process groups.
    active = wait_until_gone(token, 2.5)
    for sig, grace in ((signal.SIGINT, 3.0), (signal.SIGTERM, 3.0),
                       (signal.SIGKILL, 1.0)):
        if not active:
            break
        signal_tagged(token, sig)
        active = wait_until_gone(token, grace)
    return active


class Launcher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('STIP ROSMASTER | Workflow Launcher')
        # Compact desktop layout, designed for 1366x768 Jetson monitors.
        self.geometry('900x600')
        self.minsize(820, 560)
        self.configure(background=COLOR['bg'])
        self.protocol('WM_DELETE_WINDOW', self.on_close)
        self.events = queue.Queue()
        self.env = None
        self.active = None
        self.selected = 'system'
        self.tool_choice = {'calibration': 'thermal_align', 'tests': 'rgb_test'}
        self.map_name = tk.StringVar(value='rosmaster_map_' + datetime.datetime.now().strftime('%Y%m%d_%H%M'))
        self.map_choice = tk.StringVar(value='')
        self.map_saving = False
        self.active_navigation_map = None
        self.tool_buttons = {}
        self.start_pending = False
        self.launch_proc = None
        self.launch_log = None
        self.launch_log_fp = None
        self.watcher_proc = None
        self.stages = {}
        self.stopping = False
        self.cleanup_blocked = False
        self.launch_token = None
        self.restart_after_stop = None
        self.closing = False
        self.shutdown_signal = None
        # roslaunch sends SIGINT when the outer launcher is Ctrl+C'd. Never
        # allow a KeyboardInterrupt to abort a Tk callback mid-cleanup.
        signal.signal(signal.SIGINT, self._request_signal_shutdown)
        signal.signal(signal.SIGTERM, self._request_signal_shutdown)
        self.logs = collections.deque(maxlen=1000)
        self.last_log_pos = 0
        self.stage_rows = {}
        self.nav_buttons = {}
        self.launch_time = 0.0
        self.last_stage_update = 0.0
        self.build_ui()
        threading.Thread(target=self.load_environment, daemon=True).start()
        self.after(350, self.tick)

    def build_ui(self):
        # Compact rounded dashboard with no external theme dependencies.
        header = tk.Frame(self, bg=COLOR['bg'], height=49)
        header.pack(fill='x', padx=15, pady=(9, 5))
        header.pack_propagate(False)
        left_title = tk.Frame(header, bg=COLOR['bg'])
        left_title.pack(side='left', fill='y')
        tk.Label(left_title, text='STIP ROSMASTER', bg=COLOR['bg'], fg=COLOR['text'],
                 font=('DejaVu Sans', 15, 'bold')).pack(anchor='w')
        tk.Label(left_title, text='Workflow launcher', bg=COLOR['bg'], fg=COLOR['muted'],
                 font=('DejaVu Sans', 9)).pack(anchor='w')
        self.top_status = tk.Label(header, text='INITIALIZING  ●', bg=COLOR['bg'],
                                   fg=COLOR['yellow'], font=('DejaVu Sans', 10, 'bold'))
        self.top_status.pack(side='right', padx=6, pady=8)

        main = tk.Frame(self, bg=COLOR['bg'])
        main.pack(fill='both', expand=True, padx=12)

        sidebar = RoundedPanel(main, height=270, radius=16, padding=10)
        sidebar.pack(side='left', fill='y', padx=(0, 8))
        sidebar.configure(width=184)
        sidebar.pack_propagate(False)
        # Keep sidebar fixed, even as the main window expands.
        sidebar.config(width=184)
        tk.Label(sidebar.body, text='WORKFLOWS', bg=COLOR['panel'], fg=COLOR['muted'],
                 font=('DejaVu Sans', 9, 'bold'), anchor='w').pack(fill='x', pady=(2, 6), padx=3)
        for key, name, desc, _, _, _ in WORKFLOWS:
            short_name = 'Navigation' if key == 'navigation' else name
            btn = RoundedButton(sidebar.body, short_name, lambda k=key: self.select(k),
                                height=31, kind='nav', align='left', radius=11, font_size=9)
            btn.pack(fill='x', pady=1)
            self.nav_buttons[key] = btn
        tk.Label(sidebar.body, text='TEST & CALIBRATE', bg=COLOR['panel'], fg=COLOR['muted'],
                 font=('DejaVu Sans', 9, 'bold'), anchor='w').pack(fill='x', pady=(9, 5), padx=3)
        for key, name in (('calibration', 'Calibration'), ('tests', 'Component Tests')):
            btn = RoundedButton(sidebar.body, name, lambda k=key: self.select(k),
                                height=30, kind='nav', align='left', radius=11, font_size=9)
            btn.pack(fill='x', pady=1)
            self.nav_buttons[key] = btn

        self.main_card = RoundedPanel(main, height=270, radius=16, padding=12)
        self.main_card.pack(side='left', fill='both', expand=True)
        self.main_panel = self.main_card.body
        head = tk.Frame(self.main_panel, bg=COLOR['panel'])
        head.pack(fill='x')
        head_text = tk.Frame(head, bg=COLOR['panel'])
        head_text.pack(side='left', fill='x', expand=True)
        self.workflow_title = tk.Label(head_text, fg=COLOR['text'], bg=COLOR['panel'],
                                       font=('DejaVu Sans', 14, 'bold'), anchor='w')
        self.workflow_title.pack(fill='x')
        self.workflow_desc = tk.Label(head_text, fg=COLOR['muted'], bg=COLOR['panel'],
                                     font=('DejaVu Sans', 9), anchor='w')
        self.workflow_desc.pack(fill='x', pady=(4,0))
        self.workflow_state = tk.Label(head, text='STOPPED', bg=COLOR['panel'],
                                      fg=COLOR['muted'], font=('DejaVu Sans', 10, 'bold'))
        self.workflow_state.pack(side='right', anchor='n', pady=7)

        controls = tk.Frame(self.main_panel, bg=COLOR['panel'])
        controls.pack(fill='x', pady=(9, 7))
        self.start_btn = self.action_button(controls, 'Start workflow', self.start_selected, 'primary', 135)
        self.stop_btn = self.action_button(controls, 'Stop', self.stop_active, 'danger', 78)
        self.restart_btn = self.action_button(controls, 'Restart workflow', self.restart_selected, 'ghost', 142)
        self.cmd_label = tk.Label(self.main_panel, bg=COLOR['panel'], fg=COLOR['muted'],
                                  anchor='w', font=('DejaVu Sans Mono', 9))
        self.cmd_label.pack(fill='x', pady=(0, 6))

        self.stage_title = tk.Label(self.main_panel, text='MAIN SYSTEM STAGES',
                                    bg=COLOR['panel'], fg=COLOR['muted'], anchor='w',
                                    font=('DejaVu Sans', 9, 'bold'))
        self.stage_title.pack(fill='x', pady=(0, 4))
        # Scrollable details preserve access to all stages/tools when logs are tall.
        self.stage_panel = tk.Frame(self.main_panel, bg=COLOR['panel'])
        self.stage_panel.pack(fill='both', expand=True)
        self.detail_scroll = tk.Scrollbar(
            self.stage_panel, orient='vertical', width=11,
            bg=COLOR['surface'], troughcolor=COLOR['panel'],
            activebackground=COLOR['border'], relief='flat')
        self.detail_scroll.pack(side='right', fill='y')
        self.detail_canvas = tk.Canvas(
            self.stage_panel, bg=COLOR['panel'], bd=0, highlightthickness=0,
            yscrollcommand=self.detail_scroll.set)
        self.detail_canvas.pack(side='left', fill='both', expand=True)
        self.detail_scroll.configure(command=self.detail_canvas.yview)
        self.detail_content = tk.Frame(self.detail_canvas, bg=COLOR['panel'])
        self.detail_canvas_window = self.detail_canvas.create_window(
            (0, 0), window=self.detail_content, anchor='nw')
        self.detail_content.bind('<Configure>', self._update_detail_scroll)
        self.detail_canvas.bind('<Configure>', self._resize_detail_content)
        self.bind_all('<MouseWheel>', self._wheel_detail, add='+')
        self.bind_all('<Button-4>', self._wheel_detail, add='+')
        self.bind_all('<Button-5>', self._wheel_detail, add='+')
        self.stage_grid = tk.Frame(self.detail_content, bg=COLOR['panel'])
        self.stage_grid.pack(fill='both', expand=True)
        self.stage_grid.columnconfigure(0, weight=1, uniform='stages')
        self.stage_grid.columnconfigure(1, weight=1, uniform='stages')
        for index, (key, name) in enumerate(STAGE_NAMES):
            row = StageRow(self.stage_grid, key, name, self.restart_stage)
            row.grid(row=index // 2, column=index % 2, sticky='ew', pady=2,
                     padx=(0, 4) if index % 2 == 0 else (4, 0))
            self.stage_rows[key] = (row, row.status, row.detail, row.button)
        # Standalone test / calibration launch selector (no per-stage controls).
        self.tool_panel = tk.Frame(self.detail_content, bg=COLOR['panel'])
        self.tool_grid = tk.Frame(self.tool_panel, bg=COLOR['panel'])
        self.tool_grid.pack(fill='x')
        for c in range(2):
            self.tool_grid.columnconfigure(c, weight=1, uniform='tools')
        self.tool_info = tk.Label(self.tool_panel, text='', wraplength=490,
                                  bg=COLOR['panel'], fg=COLOR['muted'],
                                  justify='left', anchor='nw', font=('DejaVu Sans', 10))
        self.tool_info.pack(fill='x', pady=(8, 2))
        self.tool_hint = tk.Label(self.tool_panel,
                                  text='Each tool uses its existing ROS launch.\n'
                                       'Stop the active launch before starting another test.',
                                  bg=COLOR['panel'], fg=COLOR['muted'],
                                  justify='left', font=('DejaVu Sans', 9))
        self.tool_hint.pack(fill='x', pady=(4, 0))
        self._build_map_controls()
        self.no_stage_note = tk.Label(self.detail_content,
            text='This workflow is managed as one process.\n\n'
                 'Start, stop or restart it using the controls above.\n'
                 'Individual stage recovery is only available in Main System.',
            bg=COLOR['panel'], fg=COLOR['muted'], font=('DejaVu Sans', 11),
            justify='left', anchor='nw')

        # Canvas has a fixed height; Text(height=...) alone cannot resize it.
        log_card = RoundedPanel(self, height=196, radius=16, padding=9, fill=COLOR['panel'])
        log_card.pack(fill='x', padx=12, pady=(8, 10))
        log_head = tk.Frame(log_card.body, bg=COLOR['panel'])
        log_head.pack(fill='x')
        tk.Label(log_head, text='LIVE LOGS', bg=COLOR['panel'], fg=COLOR['muted'],
                 font=('DejaVu Sans', 9, 'bold')).pack(side='left')
        open_btn = RoundedButton(log_head, 'Open log folder', self.open_logs,
                                 width=120, height=23, kind='ghost', font_size=8)
        open_btn.pack(side='right')
        self.log_text = tk.Text(log_card.body, height=10, bg=COLOR['panel'],
                                fg='#cbd8e8', insertbackground=COLOR['text'],
                                font=('DejaVu Sans Mono', 9), relief='flat',
                                highlightthickness=0, bd=0,
                                state='disabled', wrap='none', cursor='arrow')
        self.log_text.pack(fill='both', expand=True, pady=(2, 0))
        self.select('system')

    def _build_map_controls(self):
        self.map_panel = tk.Frame(self.detail_content, bg=COLOR['panel'])
        self.map_section_label = tk.Label(self.map_panel, text='', bg=COLOR['panel'],
                                          fg=COLOR['text'], anchor='w',
                                          font=('DejaVu Sans', 11, 'bold'))
        self.map_section_label.pack(fill='x', pady=(2, 6))
        self.map_mapping_content = tk.Frame(self.map_panel, bg=COLOR['panel'])
        tk.Label(self.map_mapping_content,
                 text='Save current /map without stopping GMapping',
                 bg=COLOR['panel'], fg=COLOR['muted'], anchor='w',
                 font=('DejaVu Sans', 9)).pack(fill='x')
        name_row = tk.Frame(self.map_mapping_content, bg=COLOR['panel'])
        name_row.pack(fill='x', pady=(8, 4))
        tk.Label(name_row, text='Map name:', bg=COLOR['panel'], fg=COLOR['text'],
                 font=('DejaVu Sans', 9)).pack(side='left')
        self.map_name_entry = tk.Entry(name_row, textvariable=self.map_name,
                                       relief='flat', bg=COLOR['surface'],
                                       fg=COLOR['text'], insertbackground=COLOR['text'],
                                       font=('DejaVu Sans Mono', 9), width=24)
        self.map_name_entry.pack(side='left', padx=(7, 0), ipady=5, fill='x', expand=True)
        self.map_save_button = RoundedButton(self.map_mapping_content, 'Save Map',
                                             self.save_map, width=120, height=32,
                                             kind='primary', radius=10, font_size=9)
        self.map_save_button.pack(anchor='w', pady=5)
        self.map_save_note = tk.Label(self.map_mapping_content, bg=COLOR['panel'],
                                      fg=COLOR['muted'], wraplength=430,
                                      text='Start SLAM Mapping, wait for /map, then save.',
                                      anchor='w', justify='left', font=('DejaVu Sans', 9))
        self.map_save_note.pack(fill='x', pady=(1, 6))

        self.map_nav_content = tk.Frame(self.map_panel, bg=COLOR['panel'])
        tk.Label(self.map_nav_content, text='Map for AMCL + move_base:',
                 bg=COLOR['panel'], fg=COLOR['muted'], anchor='w',
                 font=('DejaVu Sans', 9)).pack(fill='x')
        dropdown_row = tk.Frame(self.map_nav_content, bg=COLOR['panel'])
        dropdown_row.pack(fill='x', pady=7)
        self.map_dropdown = tk.OptionMenu(dropdown_row, self.map_choice, '')
        self.map_dropdown.configure(bg=COLOR['surface'], fg=COLOR['text'],
                                    activebackground=COLOR['border'],
                                    activeforeground=COLOR['text'], highlightthickness=0,
                                    relief='flat', bd=0, font=('DejaVu Sans', 10),
                                    width=20)
        self.map_dropdown['menu'].configure(bg=COLOR['surface'], fg=COLOR['text'],
                                            activebackground=COLOR['border'],
                                            activeforeground=COLOR['text'])
        self.map_dropdown.pack(side='left')
        self.map_refresh_button = RoundedButton(dropdown_row, 'Refresh',
                                                 self.refresh_map_choices,
                                                 width=84, height=30, radius=10, font_size=9)
        self.map_refresh_button.pack(side='left', padx=9)
        self.map_nav_note = tk.Label(self.map_nav_content, bg=COLOR['panel'],
                                     fg=COLOR['muted'], wraplength=450, justify='left',
                                     anchor='w', font=('DejaVu Sans', 9))
        self.map_nav_note.pack(fill='x', pady=3)
        self.map_choice.trace_add('write', lambda *_: self._refresh_command_label() if self.selected == 'navigation' else None)
        self.refresh_map_choices()

    def refresh_map_choices(self):
        maps = list_usable_maps()
        menu = self.map_dropdown['menu']
        menu.delete(0, 'end')
        for name in maps:
            menu.add_command(label=name, command=lambda n=name: self.map_choice.set(n))
        current = self.map_choice.get()
        if current not in maps:
            self.map_choice.set('my_map' if 'my_map' in maps else (maps[0] if maps else ''))
        if hasattr(self, 'map_nav_note'):
            note = ('{} usable map(s) in {}'.format(len(maps), MAP_DIR) if maps
                    else 'No valid .yaml + image map pairs. Save a SLAM map first.')
            self.map_nav_note.configure(text=note)
        if hasattr(self, 'cmd_label') and self.selected == 'navigation':
            self._refresh_command_label()

    def _refresh_command_label(self):
        item = CATALOG[self.entry_key()]
        command = 'roslaunch {} {}'.format(item[3], item[4])
        if self.entry_key() == 'navigation' and self.map_choice.get():
            command += ' map:=' + self.map_choice.get()
        self.cmd_label.configure(text=command)

    def save_map(self):
        if self.map_saving or self.stopping or self.active != 'mapping' or not self.launch_proc or self.launch_proc.poll() is not None:
            messagebox.showwarning('Mapping not running', 'Start SLAM Mapping before saving a map.')
            return
        name = self.map_name.get().strip()
        if not MAP_NAME_RE.fullmatch(name):
            messagebox.showerror('Invalid name', 'Use letters, numbers, underscores or hyphens (up to 64 characters).')
            return
        if (MAP_DIR / (name + '.yaml')).exists() or (MAP_DIR / (name + '.pgm')).exists():
            messagebox.showwarning('Map exists', 'A map with that name exists. Choose a new name to avoid overwriting it.')
            return
        self.map_saving = True
        self.map_save_note.configure(text='Saving /map; keep mapping running...')
        self.refresh_controls()
        self.log('Saving map: ' + name)
        threading.Thread(target=self._save_map_worker, args=(name,), daemon=True).start()

    def _save_map_worker(self, name):
        try:
            MAP_DIR.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(prefix='.map_saving_', dir=str(MAP_DIR)) as tmp:
                temporary = Path(tmp) / name
                result = subprocess.run(['rosrun', 'map_server', 'map_saver', '-f', str(temporary)],
                                        env=self.env, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True, timeout=35)
                pgm, yaml = temporary.with_suffix('.pgm'), temporary.with_suffix('.yaml')
                if result.returncode != 0 or not pgm.is_file() or not yaml.is_file():
                    raise RuntimeError('map_saver did not produce a complete .pgm/.yaml pair: ' + result.stdout[-400:])
                if pgm.stat().st_size < 20:
                    raise RuntimeError('Saved map image is unexpectedly small.')
                # Map saver may reference its staging directory in YAML.
                data = yaml.read_text(encoding='utf-8')
                if not re.search(r'^\s*image:', data, re.MULTILINE):
                    raise RuntimeError('Missing image field in saved map YAML.')
                data = re.sub(r'^\s*image:.*$', 'image: ' + name + '.pgm', data, count=1,
                              flags=re.MULTILINE)
                yaml.write_text(data, encoding='utf-8')
                dest_pgm, dest_yaml = MAP_DIR / pgm.name, MAP_DIR / yaml.name
                if dest_pgm.exists() or dest_yaml.exists():
                    raise RuntimeError('Map filename now exists; no file overwritten.')
                os.replace(str(pgm), str(dest_pgm))
                try:
                    os.replace(str(yaml), str(dest_yaml))
                except Exception:
                    dest_pgm.unlink(missing_ok=True)
                    raise
            self.events.put(('map_saved', name))
        except subprocess.TimeoutExpired:
            self.events.put(('map_error', 'Timed out waiting for a published /map (35 seconds).'))
        except Exception as exc:
            self.events.put(('map_error', str(exc)))

    def _update_detail_scroll(self, _event=None):
        self.detail_canvas.configure(scrollregion=self.detail_canvas.bbox('all'))

    def _resize_detail_content(self, event):
        self.detail_canvas.itemconfigure(self.detail_canvas_window,
                                         width=max(1, event.width))
        self._update_detail_scroll()

    def _wheel_detail(self, event):
        # Scroll only when pointer is over the details; leave logs/other UIs alone.
        canvas = self.detail_canvas
        if not canvas.winfo_ismapped():
            return
        x, y = self.winfo_pointerxy()
        if not (canvas.winfo_rootx() <= x < canvas.winfo_rootx() + canvas.winfo_width()
                and canvas.winfo_rooty() <= y < canvas.winfo_rooty() + canvas.winfo_height()):
            return
        if getattr(event, 'num', None) == 4:
            step = -1
        elif getattr(event, 'num', None) == 5:
            step = 1
        else:
            delta = getattr(event, 'delta', 0)
            step = -int(delta / 120) if delta else 0
            if step == 0 and delta:
                step = -1 if delta > 0 else 1
        if step:
            canvas.yview_scroll(step, 'units')
            return 'break'

    def action_button(self, parent, text, command, kind, width):
        button = RoundedButton(parent, text, command, width=width, height=32,
                               kind=kind, radius=12, font_size=9)
        button.pack(side='left', padx=(0, 6))
        return button

    def load_environment(self):
        try:
            environment = ros_environment()
            self.events.put(('environment', environment))
        except Exception as exc:
            self.events.put(('error', 'ROS workspace setup failed: ' + str(exc)))

    def entry_key(self):
        if self.selected in TOOL_GROUPS:
            return self.tool_choice[self.selected]
        return self.selected

    def choose_tool(self, key):
        if key not in {entry[0] for entry in TOOL_GROUPS[self.selected]}:
            return
        self.tool_choice[self.selected] = key
        self.render_selection()
        self.refresh_controls()

    def render_selection(self):
        key = self.entry_key()
        item = CATALOG[key]
        self.workflow_title.config(text=item[1])
        self.workflow_desc.config(text=item[2])
        if self.selected == 'calibration':
            self.start_btn.set_text('Start calibration')
            self.restart_btn.set_text('Restart calibration')
        elif self.selected == 'tests':
            self.start_btn.set_text('Start test')
            self.restart_btn.set_text('Restart test')
        else:
            self.start_btn.set_text('Start workflow')
            self.restart_btn.set_text('Restart workflow')
        self._refresh_command_label()
        for k, btn in self.nav_buttons.items():
            btn.set_selected(k == self.selected)
        self.stage_grid.pack_forget()
        self.no_stage_note.pack_forget()
        self.tool_panel.pack_forget()
        self.map_panel.pack_forget()
        self.map_mapping_content.pack_forget()
        self.map_nav_content.pack_forget()
        self.detail_canvas.yview_moveto(0)
        if self.selected == 'mapping':
            self.stage_title.config(text='SLAM  /  MAP STORAGE')
            self.map_section_label.configure(text='Save Occupancy Map')
            self.map_panel.pack(fill='x', pady=4)
            self.map_mapping_content.pack(fill='x')
        elif self.selected == 'navigation':
            self.stage_title.config(text='NAVIGATION  /  MAP SELECTION')
            self.refresh_map_choices()
            self.map_section_label.configure(text='Load Saved Map')
            self.map_panel.pack(fill='x', pady=4)
            self.map_nav_content.pack(fill='x')
        elif self.selected == 'system':
            self.stage_title.config(text='MAIN SYSTEM  /  STAGE HEALTH')
            self.stage_grid.pack(fill='both', expand=True)
        elif self.selected in TOOL_GROUPS:
            self.stage_title.config(text=('CALIBRATION  /  SELECT A TOOL' if self.selected == 'calibration'
                                          else 'COMPONENT TESTS  /  SELECT A TOOL'))
            for child in self.tool_grid.winfo_children():
                child.destroy()
            self.tool_buttons = {}
            for index, entry in enumerate(TOOL_GROUPS[self.selected]):
                tool_key = entry[0]
                button = RoundedButton(self.tool_grid, entry[1],
                                       lambda k=tool_key: self.choose_tool(k),
                                       height=31, kind='nav', align='left', radius=10,
                                       font_size=9)
                button.grid(row=index // 2, column=index % 2, sticky='ew',
                            padx=(0, 5) if index % 2 == 0 else (5, 0), pady=2)
                button.set_selected(tool_key == key)
                self.tool_buttons[tool_key] = button
            self.tool_info.config(text='WINDOW: {}  •  {}'.format(item[5], item[2]))
            self.tool_panel.pack(fill='both', expand=True)
        else:
            self.stage_title.config(text='WORKFLOW DETAILS')
            self.no_stage_note.pack(fill='x', pady=12)

    def select(self, key):
        if key not in WORKFLOW and key not in TOOL_GROUPS:
            return
        self.selected = key
        self.render_selection()
        self.refresh_controls()

    def _request_signal_shutdown(self, signum, _frame):
        # Python signal handler: no Tk calls or worker sleeps in signal context.
        self.shutdown_signal = signum

    def refresh_controls(self):
        alive = self.launch_proc is not None and self.launch_proc.poll() is None
        running = self.launch_proc is not None and (alive or self.stopping or self.cleanup_blocked)
        same = running and self.active == self.entry_key()
        state = ('STOPPING' if self.stopping else
                 'CLEANUP FAILED' if self.cleanup_blocked else
                 'RUNNING' if running else 'STOPPED')
        self.workflow_state.config(text=(state if same or not running else
                                         '{} ACTIVE'.format(CATALOG[self.active][1].upper())),
                                   fg=COLOR['red'] if self.cleanup_blocked else
                                      COLOR['yellow'] if self.stopping else
                                      COLOR['green'] if same else COLOR['muted'])
        ready = self.env is not None and not self.stopping and not self.start_pending and not self.closing
        self.map_save_button.set_enabled(self.selected == 'mapping' and self.active == 'mapping'
                                         and alive and not self.map_saving and not self.stopping)
        self.map_dropdown.configure(state=('disabled' if running or self.start_pending else 'normal'))
        self.map_refresh_button.set_enabled(not running and not self.start_pending)
        self.start_btn.set_enabled(ready and not running)
        # Allow pressing Stop again if the previous cleanup was incomplete.
        self.stop_btn.set_enabled(running and not self.stopping and not self.map_saving)
        self.restart_btn.set_enabled(same and alive and not self.stopping and not self.cleanup_blocked and not self.map_saving)
        system_online = same and alive and self.selected == 'system' and not self.stopping
        for key, (_, _, _, button) in self.stage_rows.items():
            status = self.stages.get(key, {}).get('state', '')
            permitted = bool(system_online and status not in ('WAITING', 'WAITING_FOR_BRINGUP',
                                                              'STARTING', 'RETRYING', 'RESTARTING'))
            if key == 'bringup':
                permitted = bool(system_online and status in ('ERROR', 'BLOCKED'))
            button.set_enabled(permitted)
        self.top_status.config(text=('●  CLEANUP REQUIRED' if self.cleanup_blocked else
                                     '●  STOPPING' if self.stopping else
                                     '●  WORKFLOW RUNNING' if running else
                                     '●  READY' if self.env else '●  ROS SETUP REQUIRED'),
                               fg=COLOR['red'] if self.cleanup_blocked else
                                  COLOR['yellow'] if self.stopping or not self.env else
                                  COLOR['green'] if running else COLOR['accent'])

    def log(self, line):
        self.log_text.configure(state='normal')
        self.log_text.insert('end', line.rstrip('\n') + '\n')
        total_lines = int(self.log_text.index('end-1c').split('.')[0])
        if total_lines > 450:
            self.log_text.delete('1.0', '170.0')
        self.log_text.see('end')
        self.log_text.configure(state='disabled')

    def _ros_nodes(self):
        probe = subprocess.run(['rosnode', 'list'], env=self.env,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True, timeout=6)
        if probe.returncode != 0:
            error_text = probe.stderr + probe.stdout
            # A stopped ROS master is normal and no node can hold ROS resources.
            if 'Unable to communicate' in error_text or 'Connection refused' in error_text:
                return set()
            raise RuntimeError('Cannot safely check ROS graph: ' + error_text.strip()[:240])
        # Our ROS-launched GUI isn't a hardware owner.
        return set(probe.stdout.splitlines()).difference({'/rosout', '/stip_workflow_launcher_gui'})

    def _ping_node(self, node):
        try:
            result = subprocess.run(['rosnode', 'ping', '-c', '1', node], env=self.env,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True, timeout=4)
            return result.returncode == 0
        except (subprocess.TimeoutExpired, OSError):
            # Unable to prove the node is dead: fail closed.
            return True

    def check_external_nodes(self):
        """Do not start alongside live nodes; clear only verifiably stale registrations."""
        abandoned = other_tagged_processes()
        if abandoned:
            raise RuntimeError('Previous STIP launcher processes still running (PIDs {}); '
                               'stop them before starting another workflow.'.format(', '.join(map(str, abandoned[:12]))))
        nodes = sorted(self._ros_nodes())
        if not nodes:
            return
        live = [name for name in nodes if self._ping_node(name)]
        if live:
            raise RuntimeError('Another ROS workflow may own hardware: ' + ', '.join(live[:12]) +
                               '. Stop that workflow or its active GUI first.')
        # A node that does not answer could be momentarily busy. Recheck once.
        time.sleep(0.6)
        live = [name for name in nodes if self._ping_node(name)]
        if live:
            raise RuntimeError('Another ROS workflow may own hardware: ' + ', '.join(live[:12]))
        # Everything else in this ROS graph is stale. rosnode cleanup removes
        # dead master registrations; it never sends a signal to running nodes.
        result = subprocess.run(['rosnode', 'cleanup', '-y'], env=self.env,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True, timeout=10)
        if result.returncode != 0 or self._ros_nodes():
            raise RuntimeError('Stale ROS registrations could not be cleaned. '
                               'Check: rosnode list; rosnode ping -c 1 /stip_camera_arm_gui. '
                               'Do not force-kill unknown ROS processes.')
        self.events.put(('watcher_log', 'Stale ROS registrations cleaned after liveness checks.'))

    def start_selected(self):
        if self.launch_proc is not None:
            messagebox.showwarning('Launch active', 'Stop the current launch first. Shared devices must not be started twice.')
            return
        if self.start_pending:
            return
        if self.env is None:
            messagebox.showerror('ROS setup', 'ROS environment is not available.')
            return
        selected = self.entry_key()
        if selected in ('arm_test', 'base_test', 'linear_calibration', 'angular_calibration', 'imu_calibration'):
            if selected == 'arm_test':
                warning = 'Camera Arm may move pan/tilt servos. Keep personnel and cables clear.'
            elif selected in ('linear_calibration', 'angular_calibration'):
                warning = ('Real robot motion test. Clear a large floor area; have a hardware emergency stop ready. '
                           'The motion will begin only after start_test is enabled in rqt_reconfigure.')
            elif selected == 'imu_calibration':
                warning = ('Keep the robot completely stationary for IMU bias capture. '
                           'This saves a calibration file; does not apply it automatically.')
            else:
                warning = 'This starts the real robot driver. Keep the robot stationary and motors safe.'
            if not messagebox.askyesno('Hardware safety', warning + '\n\nStart this test?'):
                return
        if selected == 'navigation':
            available = list_usable_maps()
            if self.map_choice.get() not in available:
                messagebox.showerror('Map unavailable', 'Select a valid saved map from the dropdown first.')
                return
        self.start_pending = True
        self.refresh_controls()
        self.log('Launcher: checking for existing ROS hardware owners...')
        threading.Thread(target=self._start_worker, args=(selected,), daemon=True).start()

    def _start_worker(self, key):
        try:
            self.check_external_nodes()
            self.events.put(('start_ok', key))
        except Exception as exc:
            self.events.put(('error', str(exc)))

    def _actually_start(self, key):
        item = CATALOG[key]
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        logfile = LOG_DIR / '{}_{}.log'.format(key, timestamp)
        fp = open(str(logfile), 'ab', buffering=0)
        token = secrets.token_hex(16)
        launch_env = dict(self.env)
        launch_env[PROCESS_MARKER] = token
        try:
            launch_cmd = ['roslaunch', item[3], item[4]]
            if key == 'navigation':
                chosen = self.map_choice.get()
                if chosen not in list_usable_maps():
                    raise RuntimeError('Chosen navigation map is no longer available.')
                launch_cmd.append('map:=' + chosen)
                self.active_navigation_map = chosen
            proc = subprocess.Popen(launch_cmd,
                                    env=launch_env, cwd=str(WORKSPACE),
                                    stdout=fp, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)
        except Exception:
            fp.close()
            raise
        self.start_pending = False
        self.launch_proc = proc
        self.launch_token = token
        self.cleanup_blocked = False
        self.launch_log_fp = fp
        self.launch_log = logfile
        self.active = key
        self.launch_time = time.monotonic()
        self.last_log_pos = 0
        self.stages = {}
        self.stopping = False
        self.log('Starting {} (PID {}). Log: {}'.format(item[1], proc.pid, logfile))
        if key == 'navigation':
            self.log('Navigation map: {}.yaml'.format(self.active_navigation_map))
        if key == 'system':
            self.watcher_proc = None
            self.last_stage_update = 0.0
        self.refresh_controls()

    def stop_active(self):
        if self.launch_proc is None or self.stopping or self.map_saving:
            return
        proc, key, token = self.launch_proc, self.active, self.launch_token
        self.stopping = True
        self.cleanup_blocked = False
        self.log('Stopping {} and checking ALL owned child processes...'.format(CATALOG[key][1]))
        self.refresh_controls()
        threading.Thread(target=self._stop_worker, args=(proc, key, token), daemon=True).start()

    def _stop_worker(self, proc, key, token):
        errors = []
        # The staged manager has its own graceful shutdown service; honor it.
        if key == 'system' and proc.poll() is None:
            try:
                subprocess.run(['rosservice', 'call', '/stip/startup/shutdown'],
                               env=self.env, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, timeout=5)
            except (subprocess.TimeoutExpired, OSError) as exc:
                errors.append('Manager shutdown service: ' + str(exc))
        try:
            remaining = stop_roslaunch(proc, token)
        except Exception as exc:
            remaining = tagged_processes(token)
            errors.append('Process cleanup: ' + str(exc))
        self.events.put(('stop_complete', (key, remaining, errors)))

    def restart_selected(self):
        if self.launch_proc is None or self.active != self.entry_key() or self.stopping or self.cleanup_blocked:
            return
        if not messagebox.askyesno('Restart launch', 'Restart the entire {} launch?'.format(CATALOG[self.entry_key()][1])):
            return
        self.restart_after_stop = self.entry_key()
        self.stop_active()

    def restart_stage(self, stage):
        if self.active != 'system' or self.selected != 'system':
            return
        if self.launch_proc is None or self.launch_proc.poll() is not None:
            return
        state = self.stages.get(stage, {}).get('state', 'UNKNOWN')
        if stage == 'bringup' and state not in ('BLOCKED', 'ERROR'):
            messagebox.showwarning('Bringup safety', 'Bringup restart is available only after a failure.')
            return
        if stage in ('bringup', 'camera', 'qr', 'fire'):
            if not messagebox.askyesno('Stage restart',
                   'Restart {} stage?\nKeep the robot stationary; dependent services may briefly be unavailable.'.format(stage.upper())):
                return
        service = ('/stip/startup/retry_bringup' if stage == 'bringup' else
                   '/stip/modules/restart_' + stage)
        self.log('Requesting stage restart via {}'.format(service))
        threading.Thread(target=self._service_worker, args=(service,), daemon=True).start()

    def _service_worker(self, service):
        try:
            result = subprocess.run(['rosservice', 'call', service], env=self.env,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True, timeout=9)
            msg = ((result.stdout or '') + '\n' + (result.stderr or '')).strip()
            self.events.put(('service', '{}: {}'.format(service, msg[-400:])))
        except Exception as exc:
            self.events.put(('error', 'Stage restart request failed: ' + str(exc)))

    def start_watcher(self):
        if self.watcher_proc is not None and self.watcher_proc.poll() is None:
            return
        if self.active != 'system' or self.launch_proc is None or self.launch_proc.poll() is not None:
            return
        if not WATCHER.is_file():
            return
        # Launch after core ROS master is up; watcher retries registrations.
        try:
            self.watcher_proc = subprocess.Popen([sys.executable, '-u', str(WATCHER)],
                                                 env=self.env, cwd=str(WORKSPACE),
                                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                                 text=True, bufsize=1, start_new_session=True)
        except OSError as exc:
            self.log('Watcher unavailable: ' + str(exc))
            return
        proc = self.watcher_proc
        threading.Thread(target=self._read_watcher, args=(proc,), daemon=True).start()

    def _read_watcher(self, proc):
        try:
            for line in proc.stdout:
                if line.startswith('@@STAGES@@'):
                    try:
                        self.events.put(('stages', json.loads(line[len('@@STAGES@@'):].strip())))
                    except json.JSONDecodeError:
                        pass
                elif line.strip():
                    self.events.put(('watcher_log', line.strip()))
        except Exception as exc:
            self.events.put(('watcher_log', 'Stage watcher stopped: ' + str(exc)))

    def stop_watcher(self):
        proc = self.watcher_proc
        self.watcher_proc = None
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        if proc is not None and proc.stdout:
            try:
                proc.stdout.close()
            except OSError:
                pass

    def update_stage_rows(self):
        for key, (_, status_widget, detail, button) in self.stage_rows.items():
            info = self.stages.get(key, {})
            state = str(info.get('state', 'WAITING'))
            txt = str(info.get('detail', 'Not started'))
            pretty_state = {'WAITING_FOR_BRINGUP': 'WAIT BRINGUP'}.get(state, state)
            status_widget.config(text=pretty_state[:18], fg=(COLOR['green'] if state == 'READY' else
                              COLOR['red'] if state in ('ERROR', 'BLOCKED') else
                              COLOR['yellow'] if state in ('STARTING', 'RETRYING', 'RESTARTING') else
                              COLOR['muted']))
            detail.config(text=txt[-115:])
        self.refresh_controls()

    def read_log(self):
        if not self.launch_log:
            return
        try:
            with open(str(self.launch_log), 'rb') as fp:
                fp.seek(self.last_log_pos)
                content = fp.read(32000)
                self.last_log_pos = fp.tell()
        except OSError:
            return
        if content:
            lines = content.decode('utf-8', 'replace').splitlines()
            for line in lines[-80:]:
                self.log(line[:330])

    def tick(self):
        # Handle terminal Ctrl+C without raising KeyboardInterrupt in Tk.
        if self.shutdown_signal is not None and not self.closing:
            self.log('Termination signal received; stopping owned ROS workflow cleanly...')
            self.closing = True
            self.restart_after_stop = None
            if self.launch_proc is not None and not self.stopping:
                self.stop_active()
        while True:
            try:
                event, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if event == 'environment':
                self.env = payload
                self.log('ROS workspace ready: {}'.format(WORKSPACE))
            elif event == 'start_ok':
                self.start_pending = False
                if not self.closing and self.launch_proc is None:
                    try:
                        self._actually_start(payload)
                    except Exception as exc:
                        self.log('ERROR: ' + str(exc))
            elif event == 'error':
                self.start_pending = False
                self.log('ERROR: ' + str(payload))
                if not self.closing:
                    messagebox.showerror('ROSMASTER Launcher', str(payload))
            elif event == 'stages':
                if self.active == 'system':
                    self.stages = payload
                    self.last_stage_update = time.monotonic()
                    self.update_stage_rows()
            elif event == 'map_saved':
                self.map_saving = False
                self.map_save_note.configure(text='Saved: {}.pgm and {}.yaml'.format(payload, payload))
                self.map_name.set('rosmaster_map_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
                self.log('Map saved: {} / {}.yaml'.format(MAP_DIR, payload))
                self.refresh_map_choices()
            elif event == 'map_error':
                self.map_saving = False
                self.map_save_note.configure(text='Save failed: ' + str(payload))
                self.log('MAP SAVE ERROR: ' + str(payload))
                if not self.closing:
                    messagebox.showerror('Save Map failed', str(payload))
            elif event == 'service':
                self.log('Service: ' + payload)
            elif event == 'watcher_log':
                self.log('Stage watcher: ' + payload)
            elif event == 'stop_complete':
                key, remaining, errors = payload
                for item in errors:
                    self.log('Shutdown warning: {}'.format(item))
                if remaining:
                    self.stopping = False
                    self.cleanup_blocked = True
                    self.restart_after_stop = None
                    self.log('ERROR: cleanup incomplete; owned process(es) remain: {}'.format(remaining))
                    self.log('Start blocked until cleanup succeeds. Use Stop again.')
                else:
                    self.log('Stopped: {} (all owned processes exited).'.format(CATALOG[key][1]))
                    self.stop_watcher()
                    try:
                        if self.launch_log_fp:
                            self.launch_log_fp.close()
                    except OSError:
                        pass
                    self.launch_proc = None
                    self.launch_token = None
                    self.launch_log_fp = None
                    self.active = None
                    self.active_navigation_map = None
                    self.stopping = False
                    self.cleanup_blocked = False
                    self.stages = {}
                    self.update_stage_rows()
                    if self.restart_after_stop and not self.closing:
                        key_to_restart = self.restart_after_stop
                        self.restart_after_stop = None
                        self.start_pending = True
                        self.log('Restarting after verified cleanup: {}'.format(CATALOG[key_to_restart][1]))
                        threading.Thread(target=self._start_worker,
                                         args=(key_to_restart,), daemon=True).start()
                self.refresh_controls()

        if self.launch_proc is not None:
            self.read_log()
            # Unexpected parent termination: still clean up any orphaned ROS
            # children before unlocking Start. No workflow auto-restart.
            if self.launch_proc.poll() is not None and not self.stopping and not self.cleanup_blocked:
                self.log('Workflow exited (code {}); checking owned children.'.format(self.launch_proc.returncode))
                self.stop_active()
            elif self.active == 'system' and not self.stopping and not self.cleanup_blocked and self.launch_proc.poll() is None and self.watcher_proc is None and time.monotonic() - self.launch_time > 2.0:
                self.start_watcher()
            elif self.active == 'system' and self.watcher_proc is not None and self.watcher_proc.poll() is not None:
                self.stop_watcher()
        self.refresh_controls()
        if self.closing and self.launch_proc is None and not self.start_pending:
            self.stop_watcher()
            self.destroy()
        else:
            self.after(350, self.tick)

    def open_logs(self):
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        try:
            subprocess.Popen(['xdg-open', str(LOG_DIR)], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        except Exception as exc:
            self.log('Cannot open logs: ' + str(exc))

    def on_close(self):
        running = self.launch_proc is not None
        if self.map_saving:
            messagebox.showwarning('Saving map', 'Wait for map saving to finish before closing.')
            return
        if self.start_pending:
            messagebox.showwarning('Preparing launch', 'Wait for the current startup check to finish before closing.')
            return
        if running and not messagebox.askyesno('Close Launcher',
                                                 'Stop the active ROSMASTER launch and close?'):
            return
        self.closing = True
        self.restart_after_stop = None
        if running and not self.stopping:
            self.stop_active()
        elif not running:
            self.stop_watcher()
            self.destroy()



def main():
    try:
        app = Launcher()
        app.mainloop()
    except tk.TclError as exc:
        print('Tkinter display unavailable: {}'.format(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
