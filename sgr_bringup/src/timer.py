#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Empty

def talker():
    pub = rospy.Publisher('timer', Empty, queue_size=10)
    rospy.init_node('timer', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
