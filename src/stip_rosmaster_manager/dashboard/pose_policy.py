"""UI/ROS pose policy. Coordinates and confidence are distinct concepts."""
import math

def usable_map_pose(pose, age_seconds=None, max_age=2.5):
    """A waypoint requires a known map-frame pose, not a covariance threshold."""
    if not isinstance(pose,dict):return False
    if age_seconds is not None and (age_seconds<0 or age_seconds>max_age):return False
    return pose.get('source') in ('tf','amcl') and all(
        isinstance(pose.get(k),(int,float)) and math.isfinite(pose[k])
        for k in ('x','y','yaw'))

def can_edit_waypoints(patrol_state):
    return patrol_state not in ('RUNNING','PAUSED')

def can_jog_arm(patrol_state,fire_hold):
    return not fire_hold and patrol_state not in ('RUNNING','PAUSED')
