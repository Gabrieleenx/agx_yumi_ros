#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64, Int64
from sensor_msgs.msg import JointState
import utils 
import pickle




def saveObjectPickle(fileName, object_):
    print('Saving data')
    file_save = open(fileName, 'wb')
    pickle.dump(object_, file_save, protocol=pickle.HIGHEST_PROTOCOL)
    file_save.close()

class Tester(object):
    def __init__(self, jointPosList, timeList):
        self.jointPosList = jointPosList
        self.timeList = timeList
        self.index = 0    
        
        self.jointState = utils.JointState()

        self.startJointPosition = np.zeros(14) 
        self.logger = utils.Logger()
        self.k = 1 # positon error gain  
        self.firstDataPoint = 0
        
        self.maxAngleVelocity = 100 * np.pi/180
        self.time = 0
        self.timeLast = rospy.Time.now().to_sec()
        self.pub = rospy.Publisher('/yumi/egm/joint_group_velocity_controller/command', Float64MultiArray, queue_size=1)
        self.nameList = ["yumi_robr_joint_1", "yumi_robr_joint_2", "yumi_robr_joint_3", "yumi_robr_joint_4", \
                        "yumi_robr_joint_5", "yumi_robr_joint_6", "yumi_robr_joint_7", "yumi_robl_joint_1", "yumi_robl_joint_2", "yumi_robl_joint_3", \
                        "yumi_robl_joint_4", "yumi_robl_joint_5", "yumi_robl_joint_6", "yumi_robl_joint_7"]
        self.jointPos = np.zeros(14)
        self.jointVel = np.zeros(14)

        self.jointOffset = np.array([-0.0155479, -0.0059908, -0.006814363, -0.02071828, -0.000571185, 0.01696876, 0.0007526933,\
                                         -0.00090386355, -0.0052244536, -0.000945309, 0.006259396, 0.003126351383, 0.0016362062, -0.0004466520])

    def callback(self, data):

        for i in range(len(self.nameList)):
            for j in range(len(data.name)):
                if self.nameList[i] == data.name[j]:
                    self.jointPos[i] = data.position[j]# + self.jointOffset[i]
                    self.jointVel[i] = data.velocity[j]
                    
        self.jointState.UpdatePose(pose=self.jointPos)
        self.jointState.UpdateVelocity(vel=self.jointVel)
        time_ = rospy.Time.now().to_sec()
        self.time += time_ - self.timeLast
        self.timeLast = time_

        if self.firstDataPoint == 0:
            jointState = self.jointState.GetJointPosition()[0:14]
            self.startJointPosition = jointState
            self.firstDataPoint = 1


        if self.index < len(self.timeList):
            if self.time > self.timeList[self.index]:
                self.time = 0
                self.index += 1
                print(self.index)
            
        if self.index >= len(self.timeList):
            self.jointState.UpdateTargetVelocity(np.zeros(14))
            self.publishVelocity()
            print('Init done')
            saveObjectPickle('DataJointPos.obj', self.logger)
            rospy.sleep(0.5)
            rospy.signal_shutdown("done")
            return

            
        if self.index == 0:
            startJointPos = self.startJointPosition[0:14]
        else:
            startJointPos = self.jointPosList[self.index-1]

        q, dq = self.calcPosVel(startJointPos, np.zeros(14),\
                    self.jointPosList[self.index], np.zeros(14), self.timeList[self.index], self.time)

        self.logger.appendJointPoseTarget(q)
        self.logger.appendJointPoseActual(self.jointState.GetJointPosition()[0:14])

        vel = dq + self.k*(q-self.jointState.GetJointPosition()[0:14])

        self.logger.appendJointVelTarget(vel)
        self.logger.appendJointVelActual(self.jointState.GetJointVelocity())

        self.jointState.UpdateTargetVelocity(vel)

        self.publishVelocity()



    def publishVelocity(self):
        msg = Float64MultiArray()
        # Left Arm, Right Arm
        msg.data = np.hstack([self.jointState.GetJointTargetVelocity()[7:14], self.jointState.GetJointTargetVelocity()[0:7]]).tolist()
        self.pub.publish(msg)


    def calcPosVel(self, qi, dqi, qf, dqf, tf, t): # outputs target position and velocity 
        num = np.shape(qi)[0]
        q = np.zeros(num)
        dq = np.zeros(num)
        for k in range(num):
            a0 = qi[k]
            a1 = dqi[k]
            a2 = 3 * (qf[k] - (dqf[k]*tf)/3 - a1*tf*(2/3) - a0)/(tf*tf)
            a3 = (dqf[k] - (2*a2*tf + a1))/(3*tf*tf)
            q[k] = a3*t**3 + a2*t**2  + a1*t + a0
            dq[k] = 3*a3*t**2 + 2*a2*t + a1
        return q, dq


def main():

    # starting ROS node and subscribers
    rospy.init_node('InitController', anonymous=True) 
    jointPos = []
    timeList = []
    
    reset_ = np.array([0.7, -1.7, -0.8, 1.0, -2.2, 1.0, 0.0, -0.7, -1.7, 0.8, 1.0, 2.2, 1.0, 0.0])
    jointPos.append(reset_)
    timeList.append(5)

    jointPos.append(reset_ + np.array([-0.3, 0,0,0,0,0,0,0,0,0,0,0,0,0]))
    timeList.append(2)
    jointPos.append(reset_)
    timeList.append(2)

    jointPos.append(reset_ + np.array([0, 0.5,0,0,0,0,0,0,0,0,0,0,0,0]))
    timeList.append(2)
    jointPos.append(reset_)
    timeList.append(2)

    jointPos.append(reset_ + np.array([0, 0,0.5,0,0,0,0,0,0,0,0,0,0,0]))
    timeList.append(2)
    jointPos.append(reset_)
    timeList.append(2)

    jointPos.append(reset_ + np.array([0, 0,0.0,-0.5,0,0,0,0,0,0,0,0,0,0]))
    timeList.append(2)
    jointPos.append(reset_)
    timeList.append(2)

    jointPos.append(reset_ + np.array([0, 0,0.0,0,0.5,0,0,0,0,0,0,0,0,0]))
    timeList.append(2)
    jointPos.append(reset_)
    timeList.append(2)

    jointPos.append(reset_ + np.array([0, 0,0.0,0,0.0,0.5,0,0,0,0,0,0,0,0]))
    timeList.append(2)
    jointPos.append(reset_)
    timeList.append(2)

    jointPos.append(reset_ + np.array([0, 0,0.0,0,0.0,0,0.5,0,0,0,0,0,0,0]))
    timeList.append(2)
    jointPos.append(reset_)
    timeList.append(2)


    tester = Tester(jointPos, timeList)
    rospy.sleep(0.05)
    rospy.Subscriber("/yumi/egm/joint_states", JointState, tester.callback, queue_size=3)

    rospy.spin()

if __name__ == '__main__':
    main()