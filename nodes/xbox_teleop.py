#!/usr/bin/env python3
import rospy
import json

from cs7633_project.xbox_control import XboxControl
from cs7633_project.robot_control import ControllerAction
from cs7633_project.srv import ControlAction, ControlActionRequest
from cs7633_project.logger import build_action_log_msg

from std_msgs.msg import String

class XboxTeleop:
    def __init__(self) -> None:
        rospy.init_node("xbox_teleop", anonymous=True)
        self.rate = rospy.Rate(10.)
        self.controller = XboxControl()
        self.control_mode = ControlActionRequest.CONTROLLER_MANIPULATION

        # publishers
        self.log_dict_publisher = rospy.Publisher(
            "/hri/log/dict",
            String,
            queue_size=10
        )

        # srv
        self.change_robot_pose_proxy = rospy.ServiceProxy(
            "/hri/control_action", ControlAction)

    def swap_control_mode(self):
        now = rospy.Time.now()
        self.log_dict_publisher.publish(json.dumps({"source": "xbox", "msg": "change mode", "time_s": now.secs, "time_ns": now.nsecs}))
        rospy.loginfo("Changing Control Mode!")
        if self.control_mode == ControlActionRequest.CONTROLLER_MANIPULATION:
            self.control_mode = ControlActionRequest.CONTROLLER_DRIVE
            rospy.loginfo("Mode: DRIVE")
        elif self.control_mode == ControlActionRequest.CONTROLLER_DRIVE:
            self.control_mode = ControlActionRequest.CONTROLLER_MANIPULATION
            rospy.loginfo("Mode: MANIPULATION")

    def main(self):
        while not rospy.is_shutdown():
            action = self.controller.get_action(self.control_mode)
            if action == ControllerAction.CHANGE_MODE:
                self.swap_control_mode()
                rospy.loginfo("Changing Control Mode!")
                self.rate.sleep()
                continue

            if action is not None:
                # rospy.loginfo(action)
                action = int(action.value)
                state = int(self.control_mode)
                now = rospy.Time.now()
                log_dict = build_action_log_msg("xbox", now.secs, now.nsecs, action, state)
                self.log_dict_publisher.publish(json.dumps(log_dict))
                self.change_robot_pose_proxy(action, state)

            self.rate.sleep()


if __name__ == '__main__':
    XboxTeleop().main()
