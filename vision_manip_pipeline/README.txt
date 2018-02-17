 roslaunch openni_launch openni.launch 

rosrun rviz rviz

roslaunch darknet_ros darknet_ros.launch 

roslaunch gpd jb_tutorial1.launch 

rosrun rqt_reconfigure rqt_reconfigure 
	-> camera->driver->check box

rosrun vision_manip_pipeline jb_get_grasp_server.py 

rosrun vision_manip_pipeline jb_yolo_obj_det_server.py 

rosrun vision_manip_pipeline jb_conv_coord_server.py 

rosrun vision_manip_pipeline jb_vision_manip_pipeline.py backpack

