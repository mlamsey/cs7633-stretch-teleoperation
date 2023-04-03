#!/usr/bin/env python3
import rospy
from cs7633_project.srv import ControlAction, ControlActionRequest
from cs7633_project.robot_control import ControllerAction

from cs7633_project.hand_tracker import HandTracker, LANDMARK

class HandTrackerNode:
    def __init__(self) -> None:
        rospy.init_node("hand_tracker", anonymous=True)
        self.rate = rospy.Rate(10.)
        self.hand_tracker = HandTracker()
        self.control_mode = ControlActionRequest.CONTROLLER_MANIPULATION

        # srv
        self.change_robot_pose_proxy = rospy.ServiceProxy(
            "/hri/control_action", ControlAction)

    def swap_control_mode(self):
        rospy.loginfo("Changing Control Mode!")
        if self.control_mode == ControlActionRequest.CONTROLLER_MANIPULATION:
            self.control_mode = ControlActionRequest.CONTROLLER_DRIVE
            rospy.loginfo("Mode: DRIVE")
        elif self.control_mode == ControlActionRequest.CONTROLLER_DRIVE:
            self.control_mode = ControlActionRequest.CONTROLLER_MANIPULATION
            rospy.loginfo("Mode: MANIPULATION")
    
    def main(self):
        # self.control_mode = ControlActionRequest.CONTROLLER_DRIVE
        action_counter = 0
        n_actions_required = 5
        last_action = None
        while not rospy.is_shutdown():
            image, results = self.hand_tracker.get_frame()
            self.hand_tracker.show(image)

            # # direction
            # angle = self.hand_tracker.get_finger_direction(results, LANDMARK.INDEX_FINGER_TIP)
            # print(angle)
            # self.rate.sleep()
            # continue

            # # distance
            # dist = self.hand_tracker.get_finger_distance(results, LANDMARK.INDEX_FINGER_TIP, LANDMARK.THUMB_TIP)
            # print(dist)
            # self.rate.sleep()
            # continue

            # get action
            action = self.hand_tracker.get_action(results, self.control_mode)
            if action == last_action and action is not None:
                action_counter += 1
            else:
                action_counter = 0
            
            last_action = action

            if action is not None and action_counter >= n_actions_required:
                action_counter = 0
                if action == ControllerAction.CHANGE_MODE:
                    self.swap_control_mode()
                    rospy.loginfo("Changing Control Mode!")
                    # self.rate.sleep()
                    rospy.sleep(2.)
                    continue

                rospy.loginfo(action)
                # rospy.loginfo(self.control_mode)

                action = int(action.value)
                state = int(self.control_mode)
                self.change_robot_pose_proxy(action, state)

            self.rate.sleep()

if __name__ == '__main__':
    HandTrackerNode().main()
