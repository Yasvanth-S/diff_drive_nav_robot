#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('lwheel', Int32, queue_size=10)
    pub1 = rospy.Publisher('rwheel', Int32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    test = 10
    test1 = -12
    pub.publish(test)
    pub1.publish(test1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass