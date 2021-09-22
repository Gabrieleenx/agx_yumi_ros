#!/usr/bin/env python3

import numpy as np
#import tf
import utils
#import rospy
import pickle
#from sensor_msgs.msg import JointState, PointCloud
#from geometry_msgs.msg import Point32
import matplotlib.pyplot as plt
from matplotlib.offsetbox import (TextArea, DrawingArea, OffsetImage,
                                  AnnotationBbox)
import os
plt.rcParams["font.family"] = "Times New Roman"

plt.style.use('ggplot')

class DataLists:
    def __init__(self):
        self.targetTime = []
        self.targetR = []
        self.targetL = []
        self.measuredTime = []
        self.measuredR = []
        self.measuredL = []
        self.diffR = []
        self.diffL = []

def main():
    pathSim = "/home/gabriel/YumiVsSim/DataJointPosSim1sN.obj" # path to obj
    pathReal = "/home/gabriel/YumiVsSim/DataJointPosYumi1sN.obj" # path to obj

    file_load = open(pathSim, 'rb')
    savedObjSim = pickle.load(file_load)
    file_load.close()

    file_load = open(pathReal, 'rb')
    savedObjReal = pickle.load(file_load)
    file_load.close()

    numPointsSim = len(savedObjSim.jointPoseTarget)
    numPointsReal = len(savedObjReal.jointPoseTarget)

    simPos = DataLists()
    simVel = DataLists()

    yumiPos = DataLists()
    yumiVel = DataLists()

    diffTimeSim = []

    
    print('Nr points ', numPointsReal)
    for i in range(1, numPointsReal):
        diffTimeSim.append((savedObjReal.jointPoseActual[i].time - savedObjReal.jointPoseActual[i-1].time))
    print("Hz, ", 1/(np.mean(diffTimeSim)))
    

    # real yumi
    for i in range(numPointsReal):
        yumiPos.targetTime.append(savedObjReal.jointPoseTarget[i].time - savedObjReal.jointPoseTarget[0].time)
        yumiPos.targetR.append(savedObjReal.jointPoseTarget[i].data[0:7].tolist() )
        yumiPos.targetL.append(savedObjReal.jointPoseTarget[i].data[7:14].tolist() )

        yumiPos.measuredTime.append(savedObjReal.jointPoseActual[i].time - savedObjReal.jointPoseActual[0].time)
        yumiPos.measuredR.append(savedObjReal.jointPoseActual[i].data[0:7].tolist() )
        yumiPos.measuredL.append(savedObjReal.jointPoseActual[i].data[7:14].tolist() )

        yumiVel.targetTime.append(savedObjReal.jointVelTarget[i].time - savedObjReal.jointVelTarget[0].time)
        yumiVel.targetR.append(savedObjReal.jointVelTarget[i].data[0:7].tolist() )
        yumiVel.targetL.append(savedObjReal.jointVelTarget[i].data[7:14].tolist() )

        yumiVel.measuredTime.append(savedObjReal.jointVelActual[i].time - savedObjReal.jointVelActual[0].time)
        yumiVel.measuredR.append(savedObjReal.jointVelActual[i].data[0:7].tolist() )
        yumiVel.measuredL.append(savedObjReal.jointVelActual[i].data[7:14].tolist() )

        diffPos = savedObjReal.jointPoseTarget[i].data - savedObjReal.jointPoseActual[i].data
        diffVel = savedObjReal.jointVelTarget[i].data - savedObjReal.jointVelActual[i].data

        yumiPos.diffR.append(diffPos[0:7].tolist())
        yumiPos.diffL.append(diffPos[7:14].tolist())

        yumiVel.diffR.append(diffVel[0:7].tolist())
        yumiVel.diffL.append(diffVel[7:14].tolist())

    # sim yumi
    for i in range(numPointsSim):
        simPos.targetTime.append(savedObjSim.jointPoseTarget[i].time - savedObjSim.jointPoseTarget[0].time)
        simPos.targetR.append(savedObjSim.jointPoseTarget[i].data[0:7].tolist() )
        simPos.targetL.append(savedObjSim.jointPoseTarget[i].data[7:14].tolist() )

        simPos.measuredTime.append(savedObjSim.jointVelTarget[i].time - savedObjSim.jointVelTarget[0].time)
        simPos.measuredR.append(savedObjSim.jointPoseActual[i].data[0:7].tolist() )
        simPos.measuredL.append(savedObjSim.jointPoseActual[i].data[7:14].tolist() )

        simVel.targetTime.append(savedObjSim.jointVelActual[i].time - savedObjSim.jointVelTarget[0].time)
        simVel.targetR.append(savedObjSim.jointVelTarget[i].data[0:7].tolist() )
        simVel.targetL.append(savedObjSim.jointVelTarget[i].data[7:14].tolist() )

        simVel.measuredTime.append(savedObjSim.jointVelActual[i].time - savedObjSim.jointVelActual[0].time)
        simVel.measuredR.append(savedObjSim.jointVelActual[i].data[0:7].tolist() )
        simVel.measuredL.append(savedObjSim.jointVelActual[i].data[7:14].tolist() )

        diffPos = savedObjSim.jointPoseTarget[i].data - savedObjSim.jointPoseActual[i].data
        diffVel = savedObjSim.jointVelTarget[i].data - savedObjSim.jointVelActual[i].data

        simPos.diffR.append(diffPos[0:7].tolist())
        simPos.diffL.append(diffPos[7:14].tolist())

        simVel.diffR.append(diffVel[0:7].tolist())
        simVel.diffL.append(diffVel[7:14].tolist())


    #fig, ax = plt.subplots(figsize =(12, 5))
    #ax.plot(yumiPos.measuredTime[1:], diffTimeSim, '-', linewidth=1, alpha=0.8)
    #plt.show()


    fig = plt.figure(figsize =(12, 10))
    ax1 = plt.subplot(2, 1, 1)
    plt.plot(simPos.measuredTime, simPos.measuredR, '--', linewidth=1, alpha=0.8)
    plt.plot(yumiPos.measuredTime, yumiPos.measuredR, '-', linewidth=1, alpha=0.8)
    plt.plot(simPos.measuredTime, simPos.measuredL, '--', linewidth=1, alpha=0.8)
    plt.plot(yumiPos.measuredTime, yumiPos.measuredL, '-', linewidth=1, alpha=0.8)
    ax1.title.set_text('Position')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Position [rad]')

    ax2 = plt.subplot(2, 1, 2)
    plt.plot(simVel.measuredTime, simVel.measuredR, '--', linewidth=1, alpha=0.8)
    plt.plot(yumiVel.measuredTime, yumiVel.measuredR, '-', linewidth=1, alpha=0.8)
    plt.plot(simVel.measuredTime, simVel.measuredL, '--', linewidth=1, alpha=0.8)
    plt.plot(yumiVel.measuredTime, yumiVel.measuredL, '-', linewidth=1, alpha=0.8)
  
    ax2.title.set_text('Velocity')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Velocity [rad]')

    fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()








