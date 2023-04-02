#!/usr/bin/env python3
import rospy
from cs7633_project.srv import ControlAction, ControlActionRequest

from cs7633_project.hand_tracker import HandTracker, LANDMARK

class HandTrackerNode:
    def __init__(self) -> None:
        rospy.init_node("hand_tracker", anonymous=True)
        self.rate = rospy.Rate(10.)
        self.hand_tracker = HandTracker()
        self.control_mode = ControlActionRequest.CONTROLLER_DRIVE

        # srv
        self.change_robot_pose_proxy = rospy.ServiceProxy(
            "/hri/control_action", ControlAction)
    
    def main(self):
        # self.control_mode = ControlActionRequest.CONTROLLER_DRIVE
        while not rospy.is_shutdown():
            image, results = self.hand_tracker.get_frame()
            self.hand_tracker.show(image)

            # # direction
            # angle = self.hand_tracker.get_finger_direction(results, LANDMARK.INDEX_FINGER_TIP)
            # print(angle)
            # self.rate.sleep()
            # continue

            # get action
            action = self.hand_tracker.get_action(results, self.control_mode)

            if action is not None:
                rospy.loginfo(action)
                # rospy.loginfo(self.control_mode)

                action = int(action.value)
                state = int(self.control_mode)
                self.change_robot_pose_proxy(action, state)

            self.rate.sleep()

if __name__ == '__main__':
    HandTrackerNode().main()
