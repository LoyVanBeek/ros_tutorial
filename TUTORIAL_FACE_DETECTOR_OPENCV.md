set hostname
set ros master uri
rostopic list
rqt_image_view
rqt_graph

catkin_create_pkg opencv_face_detector rospy sensor_msgs cv_bridge
cd opencv_face_detector
rm -rf src
mkdir src
touch opencv_face_detector.py
chmod +x opencv_face_detector.py

open 
set shebang #!/usr/bin/env python
type some python

rosrun opencv_face_detector opencv_face_detector.py

connect to ROS:

http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)

Implement subscriber listed there

Change std_msgs string to Image

print the data

now translate to opencv image:

http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

Display the image using your own image view of opencv

Implement the HAAR classifier
