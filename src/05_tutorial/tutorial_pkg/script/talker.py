#!/usr/bin/env python
# -*-encoding:UTF-8-*-
import rospy
from std_msgs.msg import String

def talker():
    # Declaration node name
    rospy.init_node('talker', anonymous=True)
    # Declaration a publisher
    pub = rospy.Publisher('chatter', String, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Declaration publishing topic(here, String message is used)
        str_topic = String()
        # Set publishing contents
        str_topic.data = "hello world %s" % rospy.get_time()

        rospy.loginfo(str_topic.data)

        # publishing the topic
        pub.publish(str_topic)
        rate.sleep()

if __name__ == '__main__':
    talker()
