#!/usr/bin/env python
import rospy
import sys
import numpy as np
import cPickle as pickle
import tf
from jb_yolo_obj_det_client import *
from jb_get_grasp_client import *
from jb_conv_coord_client import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def main(obj_name):

    # rosservice call with object to find location of in YOLO
        # So get the bounding box of the image and 
        # calculate the center of it as a start for the grasping window?
    resp = get_obj_loc_client(obj_name)
    print resp
    x = (resp.xmax - resp.xmin)/2 + resp.xmin
    y = (resp.ymax - resp.ymin)/2 + resp.ymin
    print x,y

    # calculate the transform of the point in the raw image to
    # the grasping window in the depth point cloud!
    resp3 = conv_coord_client(x,y)    
    print resp3

    # # rosservice call to gpd with the calculated grasping window in the 
    # # point cloud to get the top grasp 
    resp2 = get_grasp_client(resp3.newX, resp3.newY, resp3.newZ)
    print resp2

    # # convert the top grasp format to a move-it useable format
    # # then use moveit to move the arm of the Baxter/PR2? in rviz/eal world?


# ==================== MAIN ====================
if __name__ == '__main__':


    if len(sys.argv) == 2:
        obj_name = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
    print "Requesting: %s"%(obj_name)

    main(obj_name)

