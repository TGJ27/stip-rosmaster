#!/usr/bin/env python3
"""Independent ROS camera-arm GUI for STIP ROSMASTER.

Starts via teleop_gui.launch; real robot bringup is included by default.
Publishes stip_rosmaster_msgs/ArmJoint to /TargetAngle only when the
robot driver is subscribed. Target display is NOT servo feedback.
"""
import sys
import signal
import tkinter as tk
from tkinter import messagebox

import rospy
from stip_rosmaster_msgs.msg import ArmJoint

BG = '#101826'
CARD = '#1b2738'
TEXT = '#edf4fc'
MUTED = '#a6b9ce'
BLUE = '#78b7ff'
STOP_REQUESTED = False


def handle_shutdown_signal(signum, _frame):
    # No Tk or rospy work in the signal handler; Tk mainloop checks this flag.
    global STOP_REQUESTED
    STOP_REQUESTED = True



class ArmApp:
    def __init__(self):
        self._closed = False
        self.pan_id = int(rospy.get_param('~pan_servo_id', 1))
        self.tilt_id = int(rospy.get_param('~tilt_servo_id', 2))
        self.limits = {
            'pan': (float(rospy.get_param('~pan_min', 0)), float(rospy.get_param('~pan_max', 180))),
            'tilt': (float(rospy.get_param('~tilt_min', 0)), float(rospy.get_param('~tilt_max', 90))),
        }
        self.target = {
            'pan': float(rospy.get_param('~pan_home', 90)),
            'tilt': float(rospy.get_param('~tilt_home', 0)),
        }
        self.step = float(rospy.get_param('~step', 2.0))
        self.duration = int(rospy.get_param('~run_time', 200))
        for axis, bounds in self.limits.items():
            if bounds[0] > bounds[1] or not bounds[0] <= self.target[axis] <= bounds[1]:
                raise ValueError('Invalid {} home or limits'.format(axis))
        self.pub = rospy.Publisher('/TargetAngle', ArmJoint, queue_size=10)
        self.root = tk.Tk()
        self.root.title('STIP ROSMASTER | Camera Arm')
        self.root.geometry('420x310')
        self.root.minsize(390, 280)
        self.root.configure(bg=BG)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        tk.Label(self.root, text='Camera Arm', bg=BG, fg=TEXT,
                 font=('DejaVu Sans', 16, 'bold')).pack(anchor='w', padx=18, pady=(15, 1))
        tk.Label(self.root, text='Manual pan / tilt  •  No movement on startup',
                 bg=BG, fg=MUTED, font=('DejaVu Sans', 9)).pack(anchor='w', padx=18)
        card = tk.Frame(self.root, bg=CARD, padx=15, pady=10)
        card.pack(fill='both', expand=True, padx=17, pady=13)
        self.labels = {}
        self.buttons = []
        for axis in ('pan', 'tilt'):
            row = tk.Frame(card, bg=CARD)
            row.pack(fill='x', pady=5)
            tk.Label(row, text=axis.upper(), width=6, anchor='w', bg=CARD, fg=TEXT,
                     font=('DejaVu Sans', 10, 'bold')).pack(side='left')
            down = self.btn(row, '− {}°'.format(self.step), lambda a=axis: self.nudge(a, -1))
            down.pack(side='left', padx=(5, 3))
            up = self.btn(row, '+ {}°'.format(self.step), lambda a=axis: self.nudge(a, 1))
            up.pack(side='left', padx=3)
            label = tk.Label(row, width=8, anchor='e', bg=CARD, fg=BLUE,
                             font=('DejaVu Sans', 10, 'bold'))
            label.pack(side='right')
            self.labels[axis] = label
        row = tk.Frame(card, bg=CARD)
        row.pack(fill='x', pady=(10, 4))
        self.home_btn = self.btn(row, 'Home 90° / 0°', self.home)
        self.home_btn.pack(side='left')
        self.limit_info = tk.Label(row, bg=CARD, fg=MUTED, font=('DejaVu Sans', 8),
                                   text='Pan 0–180° · Tilt 0–90°')
        self.limit_info.pack(side='right')
        self.status = tk.Label(self.root, bg=BG, fg=MUTED, anchor='w',
                               text='Waiting for robot driver...', font=('DejaVu Sans', 9))
        self.status.pack(fill='x', padx=18, pady=(0, 10))
        self.update_target_display()
        self.tick()

    def btn(self, parent, text, fn):
        widget = tk.Button(parent, text=text, command=fn, bg='#2b405a', fg=TEXT,
                           activebackground='#3a5472', activeforeground=TEXT,
                           borderwidth=0, relief='flat', padx=12, pady=7,
                           font=('DejaVu Sans', 9), cursor='hand2')
        self.buttons.append(widget)
        return widget

    def update_target_display(self):
        for axis, label in self.labels.items():
            label.config(text='{:.0f}°'.format(self.target[axis]))

    def ready(self):
        return not rospy.is_shutdown() and self.pub.get_num_connections() > 0

    def send(self, axis, angle):
        if not self.ready():
            self.status.config(text='Not sent: no /TargetAngle robot driver subscriber', fg='#f3bb78')
            return
        low, high = self.limits[axis]
        angle = max(low, min(high, float(angle)))
        message = ArmJoint()
        message.id = self.pan_id if axis == 'pan' else self.tilt_id
        message.run_time = self.duration
        message.angle = angle
        message.joints = []
        self.pub.publish(message)
        self.target[axis] = angle
        self.update_target_display()
        self.status.config(text='Sent {} target {:.0f}° (not measured feedback)'.format(axis, angle), fg=BLUE)

    def nudge(self, axis, sign):
        self.send(axis, self.target[axis] + sign * self.step)

    def home(self):
        if not messagebox.askyesno('Home camera arm',
                                   'Command pan and tilt to the home positions?\n'
                                   'Make sure cables and servos have clearance.', parent=self.root):
            return
        # Separate commands because /TargetAngle specifies one servo ID per message.
        if self.ready():
            self.send('pan', float(rospy.get_param('~pan_home', 90)))
            self.root.after(160, lambda: self.send('tilt', float(rospy.get_param('~tilt_home', 0))))
        else:
            self.status.config(text='Robot driver not ready; no commands sent', fg='#f3bb78')

    def tick(self):
        if STOP_REQUESTED or rospy.is_shutdown():
            self.close()
            return
        available = self.ready()
        for button in self.buttons:
            button.configure(state='normal' if available else 'disabled')
        if not available:
            self.status.configure(text='Waiting for robot driver (/TargetAngle subscriber)', fg=MUTED)
        elif self.status.cget('text').startswith('Waiting for robot driver'):
            self.status.configure(text='Robot driver connected — controls ready', fg=BLUE)
        self.root.after(500, self.tick)

    def close(self):
        if self._closed:
            return
        self._closed = True
        if not rospy.is_shutdown():
            rospy.signal_shutdown('Camera-arm GUI closed or SIGINT/SIGTERM received')
        try:
            self.root.quit()
            self.root.destroy()
        except tk.TclError:
            pass


def main():
    rospy.init_node('stip_camera_arm_gui', disable_signals=True)
    signal.signal(signal.SIGINT, handle_shutdown_signal)
    signal.signal(signal.SIGTERM, handle_shutdown_signal)
    app = ArmApp()
    app.root.mainloop()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        rospy.logerr('Camera-arm GUI failed: %s', exc)
        raise
