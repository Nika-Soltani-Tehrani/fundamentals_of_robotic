#!/usr/bin/env python3

import rospy
import math
from hw1_2.msg import distance_msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Controller():
    def __init__(self) -> None:
        rospy.init_node("controller_node", anonymous=True)
        rospy.Subscriber('/ClosestObstacle', distance_msg, self.robot_movement)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.vel_msg = Twist()
        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0
        self.min_direction = 0
        self.min_distance = 10000000

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.current_x = self.position.x
        self.current_y = self.position.y
        self.orientation = msg.pose.pose.orientation
        self.quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(self.quaternion)
        self.current_yaw = yaw

    def robot_movement(self, closest_obstacle_msg):
        #closest_obstacle_msg = rospy.wait_for_message('scan', LaserScan)
        self.min_distance = closest_obstacle_msg.distance
        self.min_direction = closest_obstacle_msg.direction
        if self.min_direction > 180:
            self.min_direction -= 180
            
        self.reverse_angle = self.min_direction - 180
        self.reverse_angle = self.reverse_angle * math.pi / 180
        
        rospy.loginfo(f"[CONTROL] min distance: {self.min_distance}, min angle: {self.reverse_angle}\n")


    def rotate_to_reverse_goal_angle(self):
        reverse_angle = self.reverse_angle
        while (abs(reverse_angle - self.current_yaw) > 0.1):
            # rotate
            rospy.loginfo(f"difference{abs(reverse_angle - self.current_yaw)}")
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0.3
            self.velocity_publisher.publish(self.vel_msg)
        
        # stop
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def move_forward(self):
        while self.min_distance > 2:
            # move forward
            self.vel_msg.linear.x = 0.2
            self.velocity_publisher.publish(self.vel_msg)
            
        # stop
        self.vel_msg.linear.x = 0
        self.velocity_publisher.publish(self.vel_msg)
        
    def move(self):
        while not rospy.is_shutdown():
            while self.min_distance > 2: 
                self.move_forward()
                self.rotate_to_reverse_goal_angle()

                if (self.min_distance < 2):
                    break

if __name__=='__main__':
    controller = Controller()
    controller.move()