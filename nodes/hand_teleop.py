#!/usr/bin/env python3
import json
import rospy
from cs7633_project.srv import ControlAction, ControlActionRequest
from cs7633_project.robot_control import ControllerAction

from cs7633_project.hand_tracker import HandTracker, LANDMARK
from cs7633_project.logger import build_action_log_msg

from std_msgs.msg import String

class HandTrackerNode:
    def __init__(self) -> None:
        rospy.init_node("hand_tracker", anonymous=True)
        self.rate = rospy.Rate(10.)
        self.hand_tracker = HandTracker()
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
            hand_in_frame = True if results.multi_hand_landmarks is not None else False

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
                    now = rospy.Time.now()
                    log_dict = build_action_log_msg("hand", now.secs, now.nsecs)
                    log_dict["msg"] = "changing mode"
                    self.log_dict_publisher.publish(json.dumps(log_dict))
                    # self.rate.sleep()
                    rospy.sleep(2.)
                    continue

                rospy.loginfo(action)
                # rospy.loginfo(self.control_mode)

                action = int(action.value)
                state = int(self.control_mode)
                now = rospy.Time.now()
                log_dict = build_action_log_msg("hand", now.secs, now.nsecs, action, state)
                log_dict["hand_in_frame"] = hand_in_frame
                self.log_dict_publisher.publish(json.dumps(log_dict))
                self.change_robot_pose_proxy(action, state)

            self.rate.sleep()

if __name__ == '__main__':
    HandTrackerNode().main()
