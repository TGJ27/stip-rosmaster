#!/usr/bin/env python3
"""Web/headless mode adapter for the established STIP staged manager.
No modification to the original stip_stage_manager.py or recovery algorithm.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import rospy
from stip_stage_manager import Manager


class HeadlessManager(Manager):
    def auto_retry_stage(self, stage):
        if stage == 'ui':
            # The original GUI window is intentionally not started. The web
            # dashboard is a distinct process and cannot be killed by ROS.
            self.ui_ready = True
            self.ui_frame_ready = True
            self.set_state('ui', 'READY', 'Headless web dashboard mode')
            return True
        return super().auto_retry_stage(stage)

    def alive_nodes(self):
        names = super().alive_nodes()
        # The inherited supervisor tests stage membership through this set.
        # The local UI stage is intentionally virtual in dashboard mode.
        return set(names) | {'/stip_inspection_ui'}

    def check_ui_first_frame(self):
        # Unlike the legacy GTK/OpenCV window, the dashboard preview is
        # decoded in the isolated web adapter, not the manager.
        return self.camera_valid


if __name__ == '__main__':
    rospy.init_node('stip_stage_manager')
    manager = HeadlessManager()
    try:manager.run()
    finally:manager.shutdown()
