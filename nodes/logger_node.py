#!/usr/bin/env python3
import json
import rospy
from cs7633_project.srv import ControlAction
from std_msgs.msg import String

from cs7633_project.logger import Logger

class LoggerNode:
    def __init__(self) -> None:
        rospy.init_node('logger')

        # guts
        self.logger = Logger("hri_logs")

        # subscriber
        self.dict_subscriber = rospy.Subscriber(
            "/hri/log/dict",
            String,
            callback=self.log_dict,
            queue_size=10
        )

        rospy.loginfo("logger launched!")
    
    def log_dict(self, data):
        self.logger.log(json.loads(data.data))

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    LoggerNode().main()