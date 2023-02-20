#!/usr/bin/env python3
import ropsy

class HandTracker:
    def __init__(self) -> None:
        ropsy.init_node("hand_tracker", anonymous=True)
    
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    HandTracker().main()
