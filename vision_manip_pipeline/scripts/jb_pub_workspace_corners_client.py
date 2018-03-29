#!/usr/bin/env python

import sys
import rospy
from vision_manip_pipeline.srv import PubWorkspace

def pub_workspace_corners_client():
    rospy.wait_for_service('pub_workspace_corners')
    try:
        pub_workspace_corners = rospy.ServiceProxy('pub_workspace_corners', PubWorkspace)
        pub_workspace_corners()
        return 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# def usage():
#     return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    # if len(sys.argv) == 3:
    #     x = int(sys.argv[1])
    #     y = int(sys.argv[2])
    # else:
    #     print usage()
    #     sys.exit(1)
    # print "Requesting %s+%s"%(x, y)
    # print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
    pub_workspace_corners_client()