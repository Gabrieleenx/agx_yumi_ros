#!/usr/bin/env python3
# should be used with https://github.com/CRIS-Chalmers/yumi
import rospy
from controller.msg import Trajectory_point, Trajectory_msg
import tf
import numpy as np

CASE = 'fixture' # 'fixture' or 'grabDLO'

def main():

    # starting ROS node and subscribers
    rospy.init_node('trajectory_test', anonymous=True) 
    pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)
    rospy.sleep(0.1)

    # --------------------------------------------------

    msg = Trajectory_msg() # message will contain list of trajectory points
    msg.header.stamp = rospy.Time.now() 
    msg.mode = 'individual' # control mode


    # ---------------
    if CASE == 'fixture':
        trajectoryPoint = Trajectory_point() # point 
        trajectoryPoint.positionRight = [0.30, -0.1, 0.04] # poition right arm [m], yumi_base_link is the origin 
        trajectoryPoint.positionLeft = [0.30, 0.1, 0.04]# poition left arm [m]
        trajectoryPoint.orientationLeft = [1,0,0,0] # orientation left arm, quaterniorns [x, y, z, w]
        trajectoryPoint.orientationRight = [1,0,0,0]# orientation right arm
        trajectoryPoint.gripperLeft = 4.0 # gripper width for the fingers [mm]
        trajectoryPoint.gripperRight = 4.0
        trajectoryPoint.pointTime = 12.0 # time to get to this point [s]

        trajectory = [trajectoryPoint]

    elif CASE == 'grabDLO':
        trajectoryPoint = Trajectory_point() # point 
        trajectoryPoint.positionRight = [0.30, -0.10, 0.001] # poition right arm [m], yumi_base_link is the origin 
        trajectoryPoint.positionLeft = [0.30, 0.10, 0.001]# poition left arm [m]
        trajectoryPoint.orientationLeft = [1,0,0,0] # orientation left arm, quaterniorns [x, y, z, w]
        trajectoryPoint.orientationRight = [1,0,0,0]# orientation right arm
        trajectoryPoint.gripperLeft = 20.0 # gripper width for the fingers [mm]
        trajectoryPoint.gripperRight = 20.0
        trajectoryPoint.pointTime = 10.0 # time to get to this point [s]

        trajectory = [trajectoryPoint]

        trajectoryPoint = Trajectory_point() # point 
        trajectoryPoint.positionRight = [0.30, -0.10, 0.001] # poition right arm [m], yumi_base_link is the origin 
        trajectoryPoint.positionLeft = [0.30, 0.10, 0.001]# poition left arm [m]
        trajectoryPoint.orientationLeft = [1,0,0,0] # orientation left arm, quaterniorns [x, y, z, w]
        trajectoryPoint.orientationRight = [1,0,0,0]# orientation right arm
        trajectoryPoint.gripperLeft = 0.0 # gripper width for the fingers [mm]
        trajectoryPoint.gripperRight = 0.0
        trajectoryPoint.pointTime = 4.0 # time to get to this point [s]
        trajectory.append(trajectoryPoint)

        trajectoryPoint = Trajectory_point() # point 
        trajectoryPoint.positionRight = [0.30, -0.10, 0.15] # poition right arm [m], yumi_base_link is the origin 
        trajectoryPoint.positionLeft = [0.30, 0.10, 0.15]# poition left arm [m]
        trajectoryPoint.orientationLeft = [1,0,0,0] # orientation left arm, quaterniorns [x, y, z, w]
        trajectoryPoint.orientationRight = [1,0,0,0]# orientation right arm
        trajectoryPoint.gripperLeft = 0.0 # gripper width for the fingers [mm]
        trajectoryPoint.gripperRight = 0.0
        trajectoryPoint.pointTime = 8.0 # time to get to this point [s]
        trajectory.append(trajectoryPoint)
    
    # ----------------------------
    msg.trajectory = trajectory
    
    pub.publish(msg)
    rospy.sleep(0.1)
    pub.publish(msg)    

    rospy.spin()


if __name__ == '__main__':
    main()

