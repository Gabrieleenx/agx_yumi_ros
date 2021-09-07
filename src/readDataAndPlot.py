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



def main():
    path = "/home/gabriel/DataJointPos.obj" # path to obj

    file_load = open(path, 'rb')
    savedObj = pickle.load(file_load)
    file_load.close()

    numPoints = len(savedObj.jointPoseTarget)

    targetTime = []
    targetRight = []
    actualTime = []
    actualRight = []
    diffRight = []

    targetVelTime = []
    targetVelRight = []
    actualVelTime = []
    actualVelRight = []
    diffVelRight = []

    for i in range(numPoints):
        targetTime.append(savedObj.jointPoseTarget[i].time)
        targetRight.append(savedObj.jointPoseTarget[i].data[0:7].tolist())
        actualTime.append(savedObj.jointPoseActual[i].time)
        actualRight.append(savedObj.jointPoseActual[i].data[0:7].tolist())
        diff =  savedObj.jointPoseActual[i].data - savedObj.jointPoseTarget[i].data
        diffRight.append(diff[0:7].tolist())

        targetVelTime.append(savedObj.jointVelTarget[i].time)
        targetVelRight.append(savedObj.jointVelTarget[i].data[0:7].tolist())
        actualVelTime.append(savedObj.jointVelActual[i].time)
        actualVelRight.append(savedObj.jointVelActual[i].data[0:7].tolist())
        diffVel = savedObj.jointVelActual[i].data - savedObj.jointVelTarget[i].data
        diffVelRight.append(diff[0:7].tolist())


    fig, ax = plt.subplots(figsize =(12, 5))
    ax.plot(targetTime, targetRight, '-', linewidth=1, alpha=0.8)
    ax.plot(actualTime, actualRight, '-', linewidth=1, alpha=0.8)

    plt.show()

    fig, ax = plt.subplots(figsize =(12, 5))
    ax.plot(targetTime, diffRight, '-', linewidth=1, alpha=0.8)

    plt.show()

    fig, ax = plt.subplots(figsize =(12, 5))
    ax.plot(targetVelTime, targetVelRight, '-', linewidth=1, alpha=0.8)
    ax.plot(actualVelTime, actualVelRight, '-', linewidth=1, alpha=0.8)

    plt.show()

    fig, ax = plt.subplots(figsize =(12, 5))
    ax.plot(targetVelTime, diffVelRight, '-', linewidth=1, alpha=0.8)

    plt.show()


if __name__ == '__main__':
    main()








