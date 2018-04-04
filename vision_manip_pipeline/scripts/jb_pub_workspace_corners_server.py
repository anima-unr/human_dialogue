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
    print 'publishing cube: {}'.format(workspace)
        # rate.sleep()

    pub_cube(pub, req.pos, req.ori)
    print 'publishing grasp: {}'.format( req.pos)

    return []

def pub_workspace_corners_server():
    rospy.init_node('pub_workspace_corners_server')
    s = rospy.Service('pub_workspace_corners', PubWorkspace, handle_pub_workspace_corners)
    print "Ready to publish workspace corners."
    rospy.spin()


def pub_cube(pub, pos, ori):

    idx = 1000

    cube = Marker()
    # TODO_PR2_TOPIC_CHANGE
    cube.header.frame_id = "/camera_depth_optical_frame"
    # cube.header.frame_id = "/head_mount_kinect_rgb_optical_frame"
    cube.header.stamp = rospy.Time.now()
    cube.type = cube.CUBE
    cube.action = cube.ADD
    cube.pose.position.x = pos[0]
    cube.pose.position.y = pos[1]
    cube.pose.position.z = pos[2]
    cube.pose.orientation.w = ori[0]
    cube.pose.orientation.x = ori[1]
    cube.pose.orientation.y = ori[2]
    cube.pose.orientation.z = ori[3]
 
    cube.ns = 'cube'
    cube.scale.x = 0.1
    cube.scale.y = 0.1
    cube.scale.z = 0.1
    cube.color.b = 1.0
    cube.color.r = float(idx)/15
    cube.color.a = 1.0
    cube.id = idx
    rospy.loginfo('\nCube Pos:\n\tx: {}\ty: {}\tz: {}'.format(cube.pose.position.x, cube.pose.position.y, cube.pose.position.z))
    rospy.loginfo('\nCube Ori:\n\tx: {}\ty: {}\tz: {}'.format(cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z))
    pub.publish(cube)
    # return cube


if __name__ == "__main__":
    pub_workspace_corners_server()