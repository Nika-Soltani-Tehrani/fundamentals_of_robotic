#!/usr/bin/python3
import rospy
import random
from hw0.msg import proximity


def talker():
    pub = rospy.Publisher('distance', proximity, queue_size=10)
    rospy.init_node('distance_sensor', anonymous=True)
    rate = rospy.Rate(1)  # Hz

    while not rospy.is_shutdown():
        msg = proximity()
        msg.up = random.randint(10, 200)
        msg.down = random.randint(10, 200)
        msg.left = random.randint(10, 200)
        msg.right = random.randint(10, 200)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__=="__main__":
    talker()