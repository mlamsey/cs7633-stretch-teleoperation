#!/usr/bin/env python3
import rospy

from cs7633_project.robot_control import RobotControl
from cs7633_project.hand_tracker import HandTracker

class RobotControlNode:
    def __init__(self) -> None:
        rospy.init_node("robot_control", anonymous=True)
        self.controller = RobotControl()

        # for testing
        self.hand_tracker = HandTracker()
    
    def main(self):
        while not rospy.is_shutdown():
            _, result = self.hand_tracker.get_frame()
            self.controller.get_action(result)

if __name__ == '__main__':
    RobotControlNode().main()