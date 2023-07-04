#!/usr/bin/env python3

import rospy

from hw1_2.msg import distance_msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Lidar():
    def __init__(self) -> None:
        self.closest_obstacle_distance = 100000000
        rospy.init_node("lidar", anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.distance_msg = distance_msg()
    
    def get_nearest_obstacle_pose(self):
        laser_msg = rospy.wait_for_message('scan', LaserScan)
        min_distance = 10000000000
        angle_of_min_distance = 0

        for i in range(360):
            if laser_msg.ranges[i] < min_distance:
                min_distance = laser_msg.ranges[i]
                angle_of_min_distance = i
        return angle_of_min_distance, min_distance

    
        
    def run(self):
        self.closest_obstacle = rospy.Publisher('/ClosestObstacle', distance_msg, queue_size=10)
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            min_angle, min_distance = self.get_nearest_obstacle_pose()
            self.distance_msg.direction = min_angle
            self.distance_msg.distance = min_distance
            self.closest_obstacle.publish(self.distance_msg)
            #rospy.loginfo(f"[SENSOR] min distance: {min_distance}, min angle: {min_angle}\n")
            self.rate.sleep()

if __name__=='__main__':
    lidar = Lidar()
    lidar.run()