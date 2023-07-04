#!/usr/bin/env python3
import rospy
import random
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from get_next_distance.srv import GetNextDestination, GetNextDestinationResponse


class Next_destination_calculator():
    def __init__(self) -> None:
        self.current_x = 3
        self.current_y = 3
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
    
    def get_distance(self, x, y):
        return math.sqrt((self.current_x - x)**2 + (self.current_y - y)**2)
    
    def calculate_angle(self, next_x, next_y):
        v1_theta = math.atan2(self.current_y, self.current_x)
        v2_theta = math.atan2(next_y, next_x)
        rotation_angle = (v2_theta - v1_theta) * (180.0 / math.pi)

        if rotation_angle < 0:
            rotation_angle += 360.0
        return rotation_angle

    def get_next_destination(self, req):
        next_x = 0
        next_y = 0
        self.current_x = req.current_x
        self.current_y = req.current_y
        
        rospy.loginfo(f"NEW CALL: {self.current_x, self.current_y}")
        distance = 0
        while distance < 5:
            next_x = random.randint(-10, 10)
            next_y = random.randint(-10, 10)
            distance = self.get_distance(next_x, next_y)
        
        rospy.loginfo(f"the next destination is: {next_x, next_y}")

        current_time = rospy.Time.now()
        theta = self.calculate_angle(next_x, next_y)
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        self.odom_broadcaster.sendTransform(
            (self.current_x, self.current_y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        res = GetNextDestinationResponse()
        res.next_x = next_x
        res.next_y = next_y
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(self.current_x, self.current_y, 0.), Quaternion(*odom_quat))
        self.odom_pub.publish(odom)
        return res
    
def listener():
    rospy.init_node("mission_node", anonymous=True)
    ndc = Next_destination_calculator()
    s = rospy.Service('/get_next_destination', GetNextDestination, ndc.get_next_destination)
    rospy.spin()

if __name__=='__main__':
    listener()