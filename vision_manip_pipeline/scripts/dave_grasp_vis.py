#!/usr/bin/env python

import rospy
import tf
from gpd.msg import GraspConfigList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import *
from copy import deepcopy
import math

class GraspApproach:
  def __init__(self):
    rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.GraspCallback)
    self.pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
    self.pospub = rospy.Publisher("/pose", PoseStamped, queue_size=10)
    self.index = 0

  def GraspCallback(self, grasps):
    if len(grasps.grasps) > 0:
      self.PubGraspApproach(grasps)

  def GraspToQuat(self, grasp):
    rotation = Quaternion(0,0,0,1)
    xaxis = grasp.axis
    yaxis = grasp.binormal
    zaxis = grasp.approach
    trace = xaxis.x + yaxis.y + zaxis.z
    if trace > 0:
      S = math.sqrt(trace+1.0)*2
      rotation.w = S / 4
      rotation.x = (zaxis.y - yaxis.z) / S
      rotation.y = (xaxis.z - zaxis.x) / S
      rotation.z = (yaxis.x - xaxis.y) / S
    elif ((xaxis.x > yaxis.y) and (xaxis.x > zaxis.z)):
      S = math.sqrt(1.0 + xaxis.x - yaxis.y - zaxis.z)*2
      rotation.w = (zaxis.y-yaxis.z) / S
      rotation.x = S / 4
      rotation.y = (xaxis.y + yaxis.x) / S
      rotation.z = (xaxis.z + zaxis.x) / S
    elif (yaxis.y > zaxis.z):
      S = math.sqrt(1.0 + yaxis.y - xaxis.x - zaxis.z)*2
      rotation.w = (xaxis.z-zaxis.x) / S
      rotation.x = (xaxis.y+yaxis.x) / S
      rotation.y = S / 4
      rotation.z = (yaxis.z + zaxis.y) / S
    else:
      S = math.sqrt(1.0 + zaxis.z - xaxis.x - yaxis.y)*2
      rotation.w = (yaxis.x - xaxis.y) / S
      rotation.x = (xaxis.z + zaxis.x) / S
      rotation.y = (yaxis.z + zaxis.y) / S
      rotation.z = S / 4
    return rotation

  def PubGraspFrame(self, grasp):
    marker = Marker()
    marker.header.frame_id = "camera_rgb_optical_frame"
    marker.header.stamp = rospy.Time.now()
    marker.id = 1
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 0.75
    marker.points = [grasp.bottom, self.addVector(grasp.bottom, grasp.axis)]
    self.pub.publish(marker)
    marker.color.r = 0
    marker.color.g = 1
    marker.id = 2
    marker.points = [grasp.bottom, self.addVector(grasp.bottom, grasp.binormal)]
    self.pub.publish(marker)
    marker.color.b = 1
    marker.color.g = 0
    marker.id = 3
    marker.points = [grasp.bottom, self.addVector(grasp.bottom, grasp.approach)]
    self.pub.publish(marker)


  def addVector(self, point1, point2):
    ret = Point()
    ret.x = point1.x + point2.x
    ret.y = point1.y + point2.y
    ret.z = point1.z + point2.z
    return ret

  def PubGraspApproach(self, grasps):
    base = deepcopy(grasps.grasps[0].bottom)
    vec = deepcopy(grasps.grasps[0].approach)
    marker = Marker()
    marker.header.frame_id = "camera_rgb_optical_frame"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = base
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.05
    marker.scale.y = .05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    self.pub.publish(marker)
    marker.id = 1
    approach = base
    approach.x -= 0.08*vec.x
    approach.y -= 0.08*vec.y
    approach.z -= 0.08*vec.z
    marker.pose.position = deepcopy(approach)
    marker.color.r = 1
    marker.color.g = 0
    self.pub.publish(marker)

    self.PubGraspFrame(grasps.grasps[0])

    pose = PoseStamped()
    pose.header.frame_id = "camera_rgb_optical_frame"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position = deepcopy(approach)
    # vec = grasps.grasps[0].axis
    # yaw = math.atan2(vec.y, vec.x);
    # pitch = math.atan2(math.sqrt((vec.x * vec.x) + (vec.y * vec.y)), vec.z );
    # roll = 0
    # print yaw, pitch, roll
    # pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, yaw))
    pose.pose.orientation = self.GraspToQuat(grasps.grasps[0])
    self.pospub.publish(pose)

def init():
 rospy.init_node('grasp_approach')

 p = GraspApproach()
 rospy.spin()

if __name__ == '__main__':
 init()