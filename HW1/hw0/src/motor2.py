#!/usr/bin/python3
import rospy
from hw0.msg import rotate

init_state = "north"

def print_directions(motor_data):
    rospy.loginfo(motor_data)


def listener():
    rospy.init_node('motor2', anonymous=True)
    rospy.Subscriber("motor2_control", rotate, print_directions)
    rospy.spin()


if __name__=="__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
