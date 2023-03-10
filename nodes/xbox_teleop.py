#!/usr/bin/env python3
import rospy

from cs7633_project.xbox_control import XboxControl
from cs7633_project.srv import ControlAction, ControlActionRequest

class XboxTeleop:
    def __init__(self) -> None:
        rospy.init_node("xbox_teleop", anonymous=True)
        self.rate = rospy.Rate(10.)
        self.controller = XboxControl()

        # srv
        self.change_robot_pose_proxy = rospy.ServiceProxy(
            "/hri/control_action", ControlAction)

    def main(self):
        while not rospy.is_shutdown():
            action = self.controller.get_action(ControlActionRequest.CONTROLLER_MANIPULATION)
            rospy.loginfo(action)

            action = int(action.value)
            state = int(ControlActionRequest.CONTROLLER_MANIPULATION)
            self.change_robot_pose_proxy(action, state)

            self.rate.sleep()


if __name__ == '__main__':
    XboxTeleop().main()
