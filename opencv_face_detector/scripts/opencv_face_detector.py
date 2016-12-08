#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospkg

bridge = CvBridge()

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get the file path for rospy_tutorials
pkg_path = rospack.get_path('opencv_face_detector')

face_cascade = cv2.CascadeClassifier(pkg_path + '/haarcascade_frontalface_alt.xml')

def callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    # resize image
    cv_image = cv2.resize(cv_image, (0,0), fx=0.5, fy=0.5) 

    cv2.imshow("Image", cv_image)

    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    cv2.imshow("Gray Image", gray_image)

    faces = face_cascade.detectMultiScale(gray_image, 1.3, 5)
    for (x, y, w, h) in faces:
        cv2.rectangle(cv_image , (x,y), (x+w,y+h), (255,0,0), 2)

    cv2.imshow("Image faces", cv_image)
    cv2.waitKey(1)

if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('face_detector', anonymous=True)

    rospy.Subscriber("image", Image, callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
