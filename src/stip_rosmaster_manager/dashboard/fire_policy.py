"""Safety policy for dashboard interpretation of original fire ROS signals.

Off means do not latch speculative fire-inspection pause requests.
Confirmed alarms remain interlocked until original ROS package deasserts them.
No policy transition automatically restarts robot motion.
"""

def evaluate_fire_hold(monitoring_enabled, pause_requested, confirmed, inspection_state='UNKNOWN'):
    if confirmed:
        return True, 'CONFIRMED_FIRE'
    if not monitoring_enabled:
        return False, ''
    if not pause_requested:
        return False, ''
    if str(inspection_state).startswith('UNVERIFIED_'):
        return True, 'MONITORING_UNAVAILABLE'
    return True, 'FIRE_INSPECTION_PAUSE'
