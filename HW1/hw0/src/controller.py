#!/usr/bin/python3
import rospy
from hw0.msg import proximity, rotate

init_state = "north"

def calculate_next_action(distance_data):
    up_dist = distance_data.up
    down_dist = distance_data.down
    left_dist = distance_data.left
    right_dist = distance_data.right
    min_dist = min(up_dist, down_dist, left_dist, right_dist)
    if min_dist == up_dist:
        degree = 180
        direction = "clockwise"
    if min_dist == down_dist:
        degree = 0
        direction = "clockwise"
    if min_dist == left_dist:
        degree = 90
        direction = "clockwise"
    if min_dist == right_dist:
        degree = 90
        direction = "anti-clockwise"
    publish_direction(degree, direction)
    update_current_state(degree, direction)


def publish_direction(degree, direction):
    motor1_pub = rospy.Publisher('motor1_control', rotate, queue_size=10)
    motor2_pub = rospy.Publisher('motor2_control', rotate, queue_size=10)
    msg = rotate()
    msg.degree = degree
    msg.direction = direction
    rospy.loginfo("in the controller node")
    motor1_pub.publish(msg)
    motor2_pub.publish(msg)


def update_current_state(degree, direction):
    global init_state
    # north
    if (init_state == "north") and (degree == 90) and (direction == 'clockwise'):
        init_state = "east"
    if (init_state == "north") and (degree == 90) and (direction == 'anti-clockwise'):
        init_state = "west"
    if (init_state == "north") and (degree == 180) and (direction == 'clockwise'):
        init_state = "south"

    # south
    if (init_state == "south") and (degree == 90) and (direction == 'clockwise'):
        init_state = "west"
    if (init_state == "south") and (degree == 90) and (direction == 'anti-clockwise'):
        init_state = "east"
    if (init_state == "south") and (degree == 180) and (direction == 'clockwise'):
        init_state = "north"

    # east
    if (init_state == "east") and (degree == 90) and (direction == 'clockwise'):
        init_state = "south"
    if (init_state == "east") and (degree == 90) and (direction == 'anti-clockwise'):
        init_state = "north"
    if (init_state == "east") and (degree == 180) and (direction == 'clockwise'):
        init_state = "west"   

    # west
    if (init_state == "west") and (degree == 90) and (direction == 'clockwise'):
        init_state = "north"
    if (init_state == "west") and (degree == 90) and (direction == 'anti-clockwise'):
        init_state = "south"
    if (init_state == "west") and (degree == 180) and (direction == 'clockwise'):
        init_state = "east"  


def listener():
    rospy.init_node('controller', anonymous=True)
    rospy.Subscriber("distance", proximity, calculate_next_action)
    rospy.spin()


if __name__=="__main__":
    listener()
