#!/usr/bin/env python

import collections

from copy import deepcopy

import rospy

import tf
import cv2
import cv_bridge
import rospkg
import tf
import sys, argparse

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

from std_msgs.msg import Header

from sensor_msgs.msg import (
    Image,
    JointState,
)

import baxter_interface

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from unr_manipulation.srv import *

import threading as thread
import pdb

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)
# Pick and place enum
STATE = enum('APPROACHING', 'PICKING', 'PICKED', 'PLACING', 'PLACED', 'NEUTRAL', 'IDLE')

class PickPlace(object):
    def __init__(self, limb):
        self.side = limb
        self._limb = baxter_interface.Limb(limb)

        circle_io = baxter_interface.DigitalIO(limb + '_lower_button')
        dash_io = baxter_interface.DigitalIO(limb + '_upper_button')
        self.calibrating = False
        self.object_calib = -1
        # self.objects = ['neutral', 'placemat', 'cup', 'plate', 'fork', 'spoon', 'knife', 'bowl', 'soda', 'wineglass']
        self.objects = ['neutral',  'cup', 'plate', 'bowl']
        # self.objects = ['neutral', 'cup']
        self.object_pick_joint_angles = dict()
        self.object_pick_poses = dict()
        self.object_place_joint_angles = dict()
        self.object_place_poses = dict()

        self._gripper = baxter_interface.Gripper(limb)
        self._gripper.calibrate()
        self._gripper.set_holding_force(100.0)

        ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()

        circle_io.state_changed.connect(self.CirclePressed_)

        # Variables to manage threads
        self.work_thread = None
        self.stop = False

        # state variables
        self.state = STATE.IDLE

    def CirclePressed_(self, value):
        if value and self.object_calib != -1:
            if self.pick:
                print "Calibrating Object Pick: " + self.objects[self.object_calib] + "..."
                self.object_pick_joint_angles[self.objects[self.object_calib]] = self._limb.joint_angles()
                self.object_pick_poses[self.objects[self.object_calib]] = self._limb.endpoint_pose()
                print "Calibrated"
                self.calibrating = False
            else:
                print "Calibrating Object Place: " + self.objects[self.object_calib] + "..."
                self.object_place_joint_angles[self.objects[self.object_calib]] = self._limb.joint_angles()
                self.object_place_poses[self.objects[self.object_calib]] = self._limb.endpoint_pose()
                print "Calibrated"
                self.calibrating = False

    def PickAndPlaceImpl(self, req):

        if self.stop:
            return
        print "Picking UP Object: " + req.object
        self._limb.set_joint_position_speed(0.2)
        if self.stop:
            return
        self.state = STATE.NEUTRAL
        self._limb.move_to_joint_positions(self.object_pick_joint_angles['neutral'])
        if self.stop:
            return
        self._gripper.command_position(100.0)
        self.state = STATE.APPROACHING
        if self.stop:
            return
        rospy.sleep(0.4)
        if self.stop:
            return
        self.state = STATE.PICKING
        self._limb.move_to_joint_positions(self.object_pick_joint_angles[req.object])
        if self.stop:
            return
        self._gripper.command_position(0.0)
        self.state = STATE.PICKED
        if self.stop:
            return
        rospy.sleep(0.4)
        if self.stop:
            return
        self._limb.move_to_joint_positions(self.object_pick_joint_angles['neutral'])
        if self.stop:
            return
        print "Placing Down Object:" + req.object
        self.state = STATE.PLACING
        if self.stop:
            return
        self._limb.move_to_joint_positions(self.object_place_joint_angles['neutral'])
        if self.stop:
            return
        self._limb.move_to_joint_positions(self.object_place_joint_angles[req.object])
        if self.stop:
            return
        self._gripper.command_position(100.0)
        self.state = STATE.PLACED
        if self.stop:
            return
        rospy.sleep(0.4)
        if self.stop:
            return
        self._limb.move_to_joint_positions(self.object_place_joint_angles['neutral'])
        if self.stop:
            return
        self._gripper.command_position(0.0)
        if self.stop:
            return
        self.state = STATE.IDLE

    def PickAndPlaceObject(self, req):
        # starting a thread that will handle the pick and place.
        self.stop = True
        if self.work_thread != None and self.work_thread.is_alive():
                self.work_thread.join()
        self.stop = False
        self.work_thread = thread.Thread(target=self.PickAndPlaceImpl, args=[req])
        self.work_thread.start()
        return pick_and_placeResponse(True)

    def PickAndPlaceCheck(self, req):
        # checks to see if the pick and place is in the final placed state
        check = self.state == STATE.PLACED
        if check:
            self.state = STATE.IDLE
        return pick_and_placeResponse(check)

    def PickAndPlaceState(self, req):
        # return the state of the pick and place
        return pick_and_place_stateResponse(self.state)
    def PickAndPlaceStop(self, req):
        # Stop the arm from picking and placing
        if self.work_thread == None:
            return pick_and_place_stopResponse(False)
        self.stop = True
        self.work_thread.join()
        self.stop = False
        return pick_and_place_stopResponse(True)

    def CalibrateObjects(self):
        '''
        Calibrate Objects: cup
        '''
        print "Calibrating Objects:"
        # Calibrate Cup
        for i, object in enumerate(self.objects):
            print "Move " + self.side + " limb to: " + object + " picking location and Press Circle"
            self.pick = True
            self.object_calib = i
            self.calibrating = True
            while self.calibrating and not rospy.is_shutdown():
                rospy.sleep(.1)
            self._gripper.command_position(0.0)
            print "Move " + self.side + " limb to: " + object + " Placing location and Press Circle"
            self.pick = False
            self.calibrating = True
            while self.calibrating and not rospy.is_shutdown():
                rospy.sleep(.1)
            self._gripper.command_position(100.0)

        self.object_calib = -1

    def _find_approach(self, pose, offset):
        ikreq = SolvePositionIKRequest()
        # Add 5 cm offset in Z direction
        try:
            pose['position'] = Point(x=pose['position'][0],
                                     y=pose['position'][1],
                                     z=pose['position'][2] + offset
                                     )
        except Exception:
            pose['position'] = Point(x=pose['position'].x,
                                     y=pose['position'].y,
                                     z=pose['position'].z + offset
                                     )
        approach_pose = Pose()
        approach_pose.position = pose['position']
        approach_pose.orientation = pose['orientation']

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        ikreq.pose_stamp.append(pose_req)
        resp = self._iksvc(ikreq)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def _find_jp(self, pose):
        ikreq = SolvePositionIKRequest()

        goal_pose = Pose()
        goal_pose.position = pose['position']
        goal_pose.orientation = pose['orientation']

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=goal_pose)
        ikreq.pose_stamp.append(pose_req)
        resp = self._iksvc(ikreq)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def ReadCalibration(self, filename):
        with open(filename, 'r') as f:
            for line in f:
                split = line.split('%')
                location = split[0]
                pick_or_place = location.split('_')[0]
                key = location.split('_')[1]
                position = split[1]
                if pick_or_place == 'pickAngles':
                    self.object_pick_joint_angles[key] = eval(position)
                elif pick_or_place == 'pickPose':
                    self.object_pick_poses[key] = eval(position)
                elif pick_or_place == 'placeAngles':
                    self.object_place_joint_angles[key] = eval(position)
                elif pick_or_place == 'placePose':
                    self.object_place_poses[key] = eval(position)

    def SaveCalibration(self, filename):
        f = open(filename, 'w')
        for key in self.object_pick_joint_angles:
            f.write('pickAngles_' + key + '%' + str(self.object_pick_joint_angles[key]) + '\n')
            f.write('pickPose_' + key + '%' + str(self.object_pick_poses[key]) + '\n')
        for key in self.object_place_joint_angles:
            f.write('placeAngles_' + key + '%' + str(self.object_place_joint_angles[key]) + '\n')
            f.write('placePose_' + key + '%' + str(self.object_place_poses[key]) + '\n')
        f.close()

    def PostParameters(self):
        for key in self.object_pick_poses:
            # Post param for Pick Position
            rospy.set_param('/ObjectPositions/' + key,
                [self.object_pick_poses[key]['position'].x,
                 self.object_pick_poses[key]['position'].y,
                 self.object_pick_poses[key]['position'].z])


def main():
    rospy.init_node("pick_and_place_service")

    parser = argparse.ArgumentParser(description='Process Pick and Place Command Line Arguments')
    parser.add_argument('--save', '-s', type=str)
    parser.add_argument('--read', '-r', type=str)
    args = parser.parse_args()

    rs = baxter_interface.RobotEnable()
    print("Enabling Robot")
    rs.enable()
    pp = PickPlace('right')

    # Calibrate Pickup Locations
    if args.read:
        print "Reading Position File: " + args.read
        pp.ReadCalibration(args.read)
    else:
        pp.CalibrateObjects()

    # Post Params
    pp.PostParameters();
    # Save Calibration to File
    if args.save:
        print "Saving Calibration File: " + args.save
        pp.SaveCalibration(args.save)

    # Advertise Service
    s = rospy.Service('pick_and_place_object', pick_and_place, pp.PickAndPlaceObject)

    s_2 = rospy.Service('pick_and_place_check', pick_and_place, pp.PickAndPlaceCheck)

    s_3 = rospy.Service('pick_and_place_state', pick_and_place_state, pp.PickAndPlaceState)

    s_4 = rospy.Service('pick_and_place_stop', pick_and_place_stop, pp.PickAndPlaceStop)

    print "READY to PICK and Place"
    rospy.spin()

if __name__ == '__main__':
    main()