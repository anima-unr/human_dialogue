#!/usr/bin/env python
import rospy
from gpd.msg import GraspConfigList
from gpd.msg import GraspConfig
from vision_manip_pipeline.srv import GetGrasp

# global variable to store grasps
grasps = []

# Callback function to receive grasps.
def callback(msg):
    global grasps
    grasps = msg.grasps
    # print grasps

# Handle of service
def handle_get_grasp(req):
    # global grasps


    # keep waiting until get grasp with close location......
    notFound = True
    while notFound:

        # Subscribe to the ROS topic that contains the grasps.
        sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback, queue_size = 1)
        print req

        # wait until the subscriber gets location objects from topic
        while grasps == []:
            print '.'

        print grasps

        # search for a grasp close to the object location    
        # rospy.loginfo('Top grasp was:')
        top_grasp = GraspConfig(score = 0);
        eps = 0.2
        for grasp in grasps:

            # not tested yet, but if can't launch inside from roslaunch api becuase can't set parms in launch file, then test something like this to get the graps at the specified location!
            if abs( req.x - grasp.surface.x) < eps and abs( req.y - grasp.surface.y) < eps: 
                if grasp.score > top_grasp.score:
                    top_grasp = grasp
                    notFound = False
                    print "GRASP FOUND!"

        # otherwise no close grasps found yet, so reset grasps and try again!
        if notFound:
            print "ERRRORRRRR NO CLOSE GRASP FOUND!!!!!! Trying again! \n\n\n"
            # grasps = []
            top_grasp = None
            rospy.sleep(0.1)

    # grasp was found, return it!
    print "Returning grasp with highest score [%s]"%(top_grasp)
    return top_grasp

# Server set up
def get_grasp_server():
    # Create a ROS node.
    rospy.init_node('get_grasp_server')

    # Create a service
    s = rospy.Service('get_grasp', GetGrasp, handle_get_grasp)

    # spin service   
    print "Ready to get top grasp!"
    rospy.spin()

#------------------------------------------------
if __name__ == "__main__":
    get_grasp_server()

