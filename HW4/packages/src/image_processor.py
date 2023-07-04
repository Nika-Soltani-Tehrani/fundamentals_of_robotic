#!/usr/bin/env python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator
from ultralytics.yolo.engine.results import Results
from turtlebot3_object_tracker.srv import GetObjectDetect, GetObjectDetectResponse

# ROS
import rospy
from sensor_msgs.msg import Image


class ImageProcessor:
    def __init__(self) -> None:
        # Image message
        self.image_msg = Image()
        self.label = "person"

        self.image_res = 240, 320, 3 # Camera resolution: height, width
        self.image_np = np.zeros(self.image_res) # The numpy array to pour the image data into

        # TODO: Subscribe on your robot's camera topic
        # NOTE: Make sure you use the provided listener for this subscription
        self.camera_subscriber = rospy.Subscriber("follower/camera/image", Image, self.camera_listener,)

        # TODO: Instantiate your YOLO object detector/classifier model
        self.model = YOLO("yolov8m.pt")
        # TODO: You need to update results each time you call your model
        #self.results: Results = None
        #self.results = self.model.predict(self.image_np)
        
        self.cv2_frame_size = 400, 320
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        # TODO: Setup your "human detection" service
        self.human_detection_server = rospy.Service('/get_object_detection', GetObjectDetect, self.get_object_detection)
        self.cords = None
        self.update_view()

    def camera_listener(self, msg: Image):
        self.image_msg.data = copy.deepcopy(msg.data)
        rospy.loginfo(self.image_msg.data)


    def update_view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0: # If there is no image data
                    continue

                # Convert binary image data to numpy array
                self.image_np = np.frombuffer(self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape(self.image_res)
                #self.results = self.model.predict(self.image_np)
                frame = copy.deepcopy(self.image_np)

                # TODO: You can use an "Annotator" to draw object bounding boxes on frame
                annotator = Annotator(frame)
                if self.cords:
                    annotator.box_label(self.cords)
                #for box in self.results[0].boxes:
                #    b = box.xyxy[0]
                #    c = box.cls
                #    annotator.box_label(b, self.model.names[int(c)])
                
                #frame = annotator.result()
                cv2.imshow("robot_view", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass
    
    def get_object_detection(self, req):
        self.label = req.label
        self.results = self.model(self.image_np)
        res = self.draw_annotation()
        return res
        
    
    def draw_annotation(self):
        detected = False
        self.cords = None

        for res in self.results:
            if len(res.boxes.xyxy) == 0 or res.names is None:
                continue
            if self.label == res.names[0]:
                detected = True
                box = res.boxes
                self.cords = box.xyxy[0].tolist()
        
        res = GetObjectDetectResponse()
        if detected and self.cords is not None:
            x1 = round(self.cords[0])
            y1 = round(self.cords[1])
            x2 = round(self.cords[2])
            y2 = round(self.cords[3])
            res.x_bounding_box = (x1 + x2) / 2
            res.y_bounding_box = (y1 + y2) / 2
            res.width_bounding_box = x2 - x1
            res.height_bounding_box = y2 - y1
            res.image_width = self.image_res[0]
            res.image_height = self.image_res[1]

            res.detected = detected
        
        return res


if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)

    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    rospy.spin()


