roslaunch freenect_launch freenect.launch  depth_registration:=true  /// NOOOT FOR PR2
cd /etc/ros///FOR PR2 (on the pr2)

rosrun rviz rviz

roslaunch darknet_ros darknet_ros.launch 

roslaunch gpd jb_tutorial1.launch ///NOOOOOOOOT THIS ONE ANYMORE

rosrun rqt_reconfigure rqt_reconfigure  ///NOOOOOOOOT THIS ONE ANYMORE
	-> camera->driver->check box

rosrun vision_manip_pipeline jb_yolo_obj_det_server.py 

rosrun vision_manip_pipeline jb_conv_coord_server //NOT THE .py here, we want c version!

rosrun vision_manip_pipeline jb_get_grasp_server.py 

rosrun vision_manip_pipeline jb_pub_workspace_corners_server.py 

///////
//NEW THINGGGSSS TO DO PROCESSING ON MY MAHCIKNE INSTEAD OF PR2 //
cd ~/freenect_headless; roslaunch freenect-dave.launch     /// THIS IF pr2 
cd ~/freenect_headless; roslaunch freenect-dave_localKinect.launch     /// THIS IF local KINECT!
rosrun vision_manip_pipeline remapKinectTopicsToLocal.py 
///////


/----
// New things for ortho proj //
rosrun vision_manip_pipeline orthoProj
rosrun vision_manip_pipeline orthographic_tf_pub.py /// IF launching freenect-dave dont need this!!
/----



rosrun vision_manip_pipeline jb_vision_manip_pipeline.py <object>

