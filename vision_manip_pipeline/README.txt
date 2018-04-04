roslaunch freenect_launch freenect.launch  depth_registration:=true

rosrun rviz rviz

roslaunch darknet_ros darknet_ros.launch 

roslaunch gpd jb_tutorial1.launch ///NOOOOOOOOT THIS ONE ANYMORE

rosrun rqt_reconfigure rqt_reconfigure  ///NOOOOOOOOT THIS ONE ANYMORE
	-> camera->driver->check box

rosrun vision_manip_pipeline jb_get_grasp_server.py 

rosrun vision_manip_pipeline jb_yolo_obj_det_server.py 

rosrun vision_manip_pipeline jb_conv_coord_server.py 

rosrun vision_manip_pipeline jb_pub_workspace_corners_server.py 

rosrun vision_manip_pipeline jb_vision_manip_pipeline.py <object>

