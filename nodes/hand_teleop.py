#!/usr/bin/env python3
import rospy

from cs7633_project.hand_tracker import HandTracker

class HandTrackerNode:
    def __init__(self) -> None:
        rospy.init_node("hand_tracker", anonymous=True)
        self.hand_tracker = HandTracker()
    
    def main(self):
        while not rospy.is_shutdown():
            image, results = self.hand_tracker.get_frame()
            self.hand_tracker.show(image)

if __name__ == '__main__':
    HandTrackerNode().main()
