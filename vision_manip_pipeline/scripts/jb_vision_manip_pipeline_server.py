import rospy
import sys
import numpy as np
import cPickle as pickle
import tf
from jb_yolo_obj_det_client import *
from jb_get_grasp_client import *
from jb_conv_coord_client import *
from jb_pub_workspace_corners_client import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gpd.msg import GraspConfig
from vision_manip_pipeline.srv import VisionManip
import roslaunch

import moveit_commander
import moveit_msgs.msg
import roslib

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

# ==========================================================
def getPoseTrans(x,y,z, ori, old_frame, new_frame):

    # get the tf between kinect frame and base?
    now = rospy.Time(0)
    t = tf.TransformListener(True, rospy.Duration(10.0))
    t.waitForTransform(new_frame, old_frame, now, rospy.Duration(3.0));
    # (trans,rot) = t.lookupTransform("/torso_lift_link", "/head_mount_kinect_ir_optical_frame", now)

    pnt = PoseStamped()
    pnt.header.frame_id = old_frame
    pnt.header.stamp = now
    pnt.pose.position.x = x
    pnt.pose.position.y = y
    pnt.pose.position.z = z
    pnt.pose.orientation.x = ori['x']
    pnt.pose.orientation.y = ori['y']
    pnt.pose.orientation.z = ori['z']
    pnt.pose.orientation.w = ori['w']

    newPnt = t.transformPose(new_frame, pnt)

    print "\nTRANSFORM:"
    print newPnt
    return newPnt

# ==========================================================
def transPoint(x, y, z, old_frame, new_frame):

    # get the tf between kinect frame and base?
    now = rospy.Time(0)
    t = tf.TransformListener(True, rospy.Duration(10.0))
    t.waitForTransform(new_frame, old_frame, now, rospy.Duration(3.0));
    # (trans,rot) = t.lookupTransform("/torso_lift_link", "/head_mount_kinect_ir_optical_frame", now)

    pnt = PointStamped()
    pnt.header.frame_id = old_frame
    pnt.header.stamp = now
    pnt.point.x = x
    pnt.point.y = y
    pnt.point.z = z

    newPnt = t.transformPoint(new_frame, pnt)

    print "\nTRANSFORM:"
    print newPnt
    return newPnt

# ==========================================================
# def transPointCloud(cloud, old_frame, new_frame):

#     # get the tf between kinect frame and base?
#     now = rospy.Time(0)
#     t = tf.TransformListener(True, rospy.Duration(10.0))
#     t.waitForTransform(new_frame, old_frame, now, rospy.Duration(3.0));
#     # (trans,rot) = t.lookupTransform("/torso_lift_link", "/head_mount_kinect_ir_optical_frame", now)

#     newCloud = t.transformPoint(new_frame, cloud)

#     print "\nTRANSFORM:"
#     print newCloud
#     return newCloud

# ==========================================================
def moveArm(newPnt):
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    limb = moveit_commander.MoveGroupCommander('right_arm')

    # format pose to correct format
    pose_target = Pose()
    pose_target.orientation.x = newPnt.pose.orientation.x
    pose_target.orientation.y = newPnt.pose.orientation.y
    pose_target.orientation.z = newPnt.pose.orientation.z
    pose_target.orientation.w = newPnt.pose.orientation.w
    pose_target.position.x = newPnt.pose.position.x
    pose_target.position.y = newPnt.pose.position.y
    pose_target.position.z = newPnt.ppose.osition.z

    print pose_target

    limb.set_pose_target(pose_target)
    limb.set_num_planning_attempts(3);
    limb.set_planning_time(5.0);
    limb.set_goal_position_tolerance(0.0075)
    limb.set_goal_orientation_tolerance(0.0075)

    print("\tPlanning...")
    plan1 = limb.plan()
    rospy.sleep(10)
    # print("\tExecuting...")
    # limb.go(wait=True)
    # rospy.sleep(5)

# ==========================================================


def rotationQuat(axis, binormal, approach):

    rotation = {'w':0,'x':0,'y':0,'z':0}

    if approach.z < 0:
        if axis.x > binormal.y:
            trace = 1 + axis.x - binormal.y - approach.z
            rotation['w'] = trace
            rotation['x'] = axis.y + binormal.x
            rotation['y'] = approach.x + axis.z
            rotation['z'] = binormal.z - approach.y
        else:
            trace = 1 - axis.x + binormal.y - approach.z
            rotation['w'] = axis.y + binormal.x
            rotation['x'] = trace
            rotation['y'] = binormal.z + approach.y
            rotation['z'] = approach.x - axis.z
    else:
        if axis.x < (-binormal.y):
            trace = 1 - axis.x - binormal.y + approach.z
            rotation['w'] = approach.x + axis.z
            rotation['x'] = binormal.z + approach.y
            rotation['y'] = trace
            rotation['z'] = axis.y - binormal.x
        else:
            trace = 1 + axis.x + binormal.y + approach.z
            rotation['w'] = binormal.z - approach.y
            rotation['x'] = approach.x - axis.z
            rotation['y'] = axis.y - binormal.x
            rotation['z'] = trace

    rotation['w'] *= 0.5 / (trace **(0.5))
    rotation['x'] *= 0.5 / (trace **(0.5))
    rotation['y'] *= 0.5 / (trace **(0.5))
    rotation['z'] *= 0.5 / (trace **(0.5))

    return rotation

#====================================================


def handle_vision_manip(req):
    # Create a ROS node.
    rospy.init_node('vision_manip')
    # rospy.on_shutdown(self.shutdown)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    #launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/janelle/onr_ws/src/vision_manip/launch/jb_tutorial1.launch"])
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mzagainova/cws_hetero_gripper/src/vision_manip/launch/jb_tutorial1.launch"])

    # rosservice call with object to find location of in YOLO
        # So get the bounding box of the image and
        # calculate the center of it as a start for the grasping window?
    resp = get_obj_loc_client(req)
    print resp
    x = (resp.xmax - resp.xmin)/2 + resp.xmin
    y = (resp.ymax - resp.ymin)/2 + resp.ymin
    print x,y

    # if object not detected, then exit?
    if x == 0 and y == 0:
        print "Error: Object not detected, try again!"
        return

    # calculate the transform of the point in the raw image to
    # the grasping window in the depth point cloud!
    resp3 = conv_coord_client(x,y)
    print resp3

    # TODO: JB
    # transform the point from kinect frame to the orthographic frame (static tf projection)
    newPnt = transPoint(resp3.newX, resp3.newY, resp3.newZ, "/head_mount_kinect_rgb_optical_frame", "/test")

    # TODO: JB
    # generate the cube from the transformed point instead
    #  first set the param for the workspace based on the response?!?
    eps = 0.1
    cube = [newPnt.x - eps, newPnt.x + eps, newPnt.y - eps, newPnt.y + eps, newPnt.z - eps, newPnt.z + eps]
    # cube = [1,1.1,1,1.1,1,1.1]
    print "cube to search for graps:"
    print cube
    # if rospy.has_param("/detect_grasps/"):
        # rospy.delete_param("/detect_grasps/")
    rospy.set_param('/detect_grasps/workspace', cube)
    rospy.set_param('/detect_grasps/workspace_grasps', cube)
    # v = rospy.get_param('/detect_grasps/clustered_grasps')
    # print v
    # print "\n\n"

    # TODO: JB
    # transform the point cloud to the orthographic frame (static tf projection) ->  will probs need to change frames in launch to get cloud in correct frame!!!
    # transPointCloud(cloud, "/local/depth_registered/points", "/local/depth_registered/trans_points"):


    # relaunch the grasp stuffsssss
    launch.start()

    # # rosservice call to gpd with the calculated grasping window in the
    # # point cloud to get the top grasp
    resp2 = get_grasp_client(resp3.newX, resp3.newY, resp3.newZ)
    print resp2

    launch.shutdown()

    # IF GRASP NOT FOUND, return 0?
    if resp2 == None:
    # if resp2.grasp.score == 0:
        print "Error: No grasp found, will now return"
        return

    # # convert the top grasp format to a move-it useable format
    # # then use moveit to move the arm of the Baxter/PR2? in rviz/eal world?
    ori = rotationQuat(resp2.grasp.axis, resp2.grasp.binormal, resp2.grasp.approach);
    print ori

    # visualize the workspace and grasp.......
    pos = [resp2.grasp.surface.x, resp2.grasp.surface.y, resp2.grasp.surface.z]
    tilt = [ori['w'], ori['x'], ori['y'], ori['z']]
    print pos
    print tilt
    pub_workspace_corners_client(pos,tilt)

    # TODO: JB
    #  WILL need to transform from other frame here nowwwwwww!!!!!
    # Transform point into correct PR2 frame for motion planning etc...
    newPnt = getPoseTrans(resp3.newX, resp3.newY, resp3.newZ, ori, "/test", "/torso_lift_link")

    # # TODO: use moveit to plan to this position and orientation!
    # moveArm(newPnt)

    #return oritentation, position and point to client
    return [ori, pos, newPnt]

def vision_manip_server():
    #create ROS node
    rospy.init_node('vision_manip_server')

    s = rospy.Service('vision_manip', VisionManip, handle_vision_manip)
    rospy.spin()

if __name__ == "__main__":
    vision_manip_server()
