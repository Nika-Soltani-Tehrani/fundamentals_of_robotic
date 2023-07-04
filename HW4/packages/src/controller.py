#!/usr/bin/python3

# ROS
import rospy
from geometry_msgs.msg import Twist
import math
from turtlebot3_object_tracker.srv import GetObjectDetect, GetObjectDetectRequest


class Controller:
    def __init__(self) -> None:
        self.label = "person"
        # Use these Twists to control your robot
        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()

        # The "p" parameter for your p-controller, TODO: you need to tune this
        self.angular_vel_coef = 0.005

        # TODO: Create a service proxy for your human detection service
        rospy.wait_for_service('/get_object_detection')
        self.get_object_detection_srv = rospy.ServiceProxy('/get_object_detection', GetObjectDetect)
        # TODO: Create a publisher for your robot "cmd_vel"
        self.cmd_vel = rospy.Publisher('/follower/cmd_vel', Twist, queue_size=10)
    
    def run(self) -> None:
        
        try:
            while not rospy.is_shutdown():
                # TODO: Call your service, ride your robot
                req = GetObjectDetectRequest()
                req.label = self.label
                response = self.get_object_detection_srv(req)
                if response.detected:
                    self.move(response)
                else:
                    self.cmd_vel.publish(self.freeze)
                    rospy.loginfo("The robot is stopped")
                
                rospy.sleep(0)

        except rospy.exceptions.ROSInterruptException:
            pass
    
    def move(self, response):
        x_bounding_box, y_bounding_box, width_bounding_box, height_bounding_box, image_width, image_height = response.x_bounding_box, response.y_bounding_box, response.width_bounding_box, response.height_bounding_box, response.image_width, response.image_height
        err = (image_width / 2) - x_bounding_box
        diff_angle = math.atan2(err, image_width)
        self.move.angular.z = self.angular_vel_coef * diff_angle
        self.cmd_vel.publish(self.move)


if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    
    controller = Controller()
    controller.run()
    
