#!/usr/bin/env python
# -*-encoding:utf-8-*-
import rospy
from std_msgs.msg import String

# Callback function(automatically called when a set topic is received)
# argugment "data" has receiving topic
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)


def listener():
    # Declaration node name
    rospy.init_node('listener', anonymous=True)
    # Declaration a subscrirber(must be set subscribing topic name, topic message and callback function name)
    rospy.Subscriber('chatter', String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
