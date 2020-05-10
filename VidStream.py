#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def main():
    rospy.init_node('VidCapture', anonymous="True")
    rate = rospy.Rate(15)
    cap = cv2.VideoCapture(0)
    cap.set(3,320)
    cap.set(4,240)
    pub = rospy.Publisher('/raspicam_node/image', Image, queue_size=1)
    bridge = CvBridge()
    while not rospy.is_shutdown():
	if cap.isOpened()==True:
            ret, frame = cap.read()
      	    frame_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
	    pub.publish(frame_msg)
	    #cv2.imshow('frame',frame)
	    cv2.waitKey(1)
        rate.sleep()

if __name__ == '__main__':
    main()
