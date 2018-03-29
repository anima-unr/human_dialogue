#!/usr/bin/env python

import rospy
from gpd.msg import GraspConfigList
from gpd.msg import GraspConfig
from vision_manip_pipeline.srv import PubWorkspace
from jb_pub_workspace_corners import *

def handle_pub_workspace_corners(req):

    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10) 

    workspace = rospy.get_param("/detect_grasps/workspace")
    workspace_grasps = rospy.get_param("/detect_grasps/workspace_grasps")

    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    pub_workspace(pub, workspace, workspace_grasps)
    print 'publishing.: {}'.format(workspace)
        # rate.sleep()

    return

def pub_workspace_corners_server():
    rospy.init_node('pub_workspace_corners_server')
    s = rospy.Service('pub_workspace_corners', PubWorkspace, handle_pub_workspace_corners)
    print "Ready to publish workspace corners."
    rospy.spin()

if __name__ == "__main__":
    pub_workspace_corners_server()