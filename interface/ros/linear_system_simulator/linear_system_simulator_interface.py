#!/usr/bin/env python

import rospy
# from std_msgs.msg import String
from ${APP_INTERFACE_NAME}.msg import report_t

def callback(report):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", report.state)
    rospy.loginfo(rospy.get_caller_id())

def listener():
    rospy.init_node('linear_system_simulator_interface', anonymous=True)
    rospy.Subscriber("report", report_t, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()