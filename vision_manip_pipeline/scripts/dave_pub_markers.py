#!/usr/bin/env python

import rospy
from gpd.msg import GraspConfigList
from visualization_msgs.msg import Marker

class GraspApproach:
 def __init__(self):
   rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.GraspCallback)
   self.pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
   self.index = 0  

 def GraspCallback(self, grasps):
       if len(grasps.grasps) > 0:
               self.PubGraspApproach(grasps)

 def PubGraspApproach(self, grasps):
   base = grasps.grasps[0].bottom
   vec = grasps.grasps[0].approach
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
   marker.pose.position = approach
   marker.color.r = 1
   marker.color.g = 0
   self.pub.publish(marker)

def init():
 rospy.init_node('grasp_approach')
 p = GraspApproach()
 rospy.spin()

if __name__ == '__main__':
 init()