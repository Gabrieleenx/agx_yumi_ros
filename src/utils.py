#!/usr/bin/env python3

import numpy as np
import rospy


class JointState(object):
    def __init__(self,\
            jointPosition=np.array([1.0, -2.0, -1.2, 0.6, -2.0, 1.0, 0.0, -1.0, -2.0, 1.2, 0.6, 2.0, 1.0, 0.0]),\
            jointVelocity=np.zeros(14)):
        self.jointPosition = jointPosition # only arm not gripper
        self.jointVelocity = jointVelocity # only arm not gripper
        self.jointTargetVelocity = jointVelocity # only arm not gripper

    
    def GetJointVelocity(self):
        return np.hstack([self.jointVelocity])
    
    def GetJointTargetVelocity(self):
        return np.hstack([self.jointTargetVelocity])
    
    def GetJointPosition(self):
        return np.hstack([self.jointPosition])

    def UpdatePose(self, pose):
        self.jointPosition = pose[0:14]

    def UpdateVelocity(self, vel):
        self.jointVelocity = vel[0:14]

    def UpdateTargetVelocity(self, vel):
        self.jointTargetVelocity = vel[0:14]
        

class DataWrapper(object):
    def __init__(self, time, data):
        self.time = time
        self.data = data

class Logger(object):
    def __init__(self):
        self.jointPoseTarget = [] 
        self.jointPoseActual = [] 
        self.jointVelTarget = []
        self.jointVelActual = []

    def appendJointPoseTarget(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.jointPoseTarget.append(obj)

    def appendJointPoseActual(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.jointPoseActual.append(obj)

    def appendJointVelActual(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.jointVelActual.append(obj)

    def appendJointVelTarget(self, data):
        obj = DataWrapper(time=rospy.Time.now().to_sec(),\
                        data=data)
        self.jointVelTarget.append(obj)