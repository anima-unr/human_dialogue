#!/usr/bin/env python

import sys
import rospy
from table_task_sim.srv import *

def pick_client():
    rospy.wait_for_service('pick_service')
    try:
        add_two_ints = rospy.ServiceProxy('pick_service', PickUpObject)
        resp1 = add_two_ints(1, "meat")
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    print "result = %s"%(pick_client())
