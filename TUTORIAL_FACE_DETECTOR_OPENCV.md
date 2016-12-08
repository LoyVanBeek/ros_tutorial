# Face detector tutorial

# Join the network

Ask for the network credentials

## Set ROS MASTER URI parameters to communicate with the other laptop
Set the hostname in `/etc/hosts`

    sudo gedit /etc/hosts
    
add the following line

    192.168.26.127 athome10 
    
Try to ping the remote

    ping athome10
    
If this does not work, please ask ..

Now set the `ROS_MASTER_URI` in your `.bashrc` file:

    echo "export ROS_MASTER_URI=http://athome10:11311" >> ~/.bashrc
    
Now your environment will know where to look for the ROS master. Start a new shell and test:

    rostopic list
    
You should now see all the exposed topic in the ROS network. You can now play with all `rosnode` and `rostopic` cmds:

- http://wiki.ros.org/rostopic
- http://wiki.ros.org/rosnode

And the ros_rqt tools family:

    rqt
    rqt_graph
    
etc ..

## Get a stream of the camera and view it

Type the following in a new console:

    rqt_image_view
    
This will open a GUI where you can select a topic_name with type `sensor_msgs/Image`. Select the `image_color` stream and see if it works. It will most probably have a low rate and delayed due to wifi but that's ok.

## Now start the tutorial

We are going to make a face detector that runs on your platform and uses the remote camera. First, create a package (http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage) in your catkin workspace:

    catkin_create_pkg opencv_face_detector rospy sensor_msgs cv_bridge
    
Go to the package and create a scripts directory with an empty python file:
    
    cd opencv_face_detector
    rm -rf src
    mkdir src
    touch opencv_face_detector.py
    
Make the file executable and set python interpreter:
    
    chmod +x opencv_face_detector.py
    gedit openface_face_detector.py
    #!/usr/bin/env python
    
Now type some python here:

    print "hello world"
    
Now run the executable via the `rosrun` command:

    rosrun opencv_face_detector opencv_face_detector.py

The current python executable is not a ROS node yet, let us connect it to ROS (make it a Node). Implement the Subscriber example of the following tutorial: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)

Now change the `std_msgs/String` to `sensor_msgs/Image` and print the image. It will for sure be a lot of information.

Now interpret this binary data as an image using `opencv` and `cvBridge` as shown here: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

Display the image using your own image view of opencv: `cv2.imshow('name', image); cv2.waitKey(10)`

Now that we have the image in opencv format, we can implement a `HaarClassifier` for face detection: http://docs.opencv.org/trunk/d7/d8b/tutorial_py_face_detection.html
