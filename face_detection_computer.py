#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest

# Define global variables
global pub_Image
global pub_BoxesImage
global pub_Boxes
global face_cascade
global bridge
global current_pose
global boxes_msg

def callback(image_msg):
    global pub_Image
    global pub_BoxesImage
    global pub_Boxes
    global face_cascade
    global bridge
    global boxes_msg
    global faces
    global local_position_pub
    global goal_pose
    # Convert ros image into a cv2 image
    cv_img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    cv_gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    # Find the faces and draw a rectangle around them
    faces = face_cascade.detectMultiScale(cv_gray, 1.3, 5)
    for (x,y,w,h) in faces:
        cv_img = cv2.rectangle(cv_img, (x,y), (x+w,y+h), (255,0,0), 2)
        # For each image, we also want to publish the croped image
        crop_img = cv_img[y:y+h, x:x+w]
        crop_msg = bridge.cv2_to_imgmsg(crop_img, encoding="bgr8")
        # And the region of interest information
        pub_BoxesImage.publish(crop_msg)
        boxes_msg = RegionOfInterest()
        boxes_msg.x_offset = x
        boxes_msg.y_offset = y
        boxes_msg.height = h
        boxes_msg.width = w
        boxes_msg.do_rectify = False
        pub_Boxes.publish(boxes_msg)
    # Publish the original image with the rectangles
    mod_img = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
    pub_Image.publish(mod_img)

    
def main():
    global pub_Image
    global pub_BoxesImage
    global pub_Boxes
    global face_cascade
    global bridge
    global current_pose
    global boxes_msg
    global faces
    global local_position_pub
    global goal_pose
    # Create a node
    rospy.init_node('face_dection', anonymous='True')
    # Setup the face detector and bridge
    face_cascade = cv2.CascadeClassifier('/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_frontalface_default.xml')
    bridge = CvBridge()
    # Create publishers and subscribers
    pub_Image = rospy.Publisher('/face_detection/image', Image, queue_size=1)
    pub_BoxesImage = rospy.Publisher('/face_detection/boxes_image', Image, queue_size=1)
    pub_Boxes = rospy.Publisher('/face_detection/boxes', RegionOfInterest, queue_size=1)
    rospy.Subscriber('/raspicam_node/image', Image, callback, queue_size=1, buff_size=2**18)

    # Keep program alive until we stop it
    rospy.spin()

if __name__ == "__main__":
    main()
