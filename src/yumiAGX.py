#!/usr/bin/env python3.8
# source /opt/Algoryx/AGX-2.31.1.0/setup_env.bash
'''
rostopic pub /yumi/egm/joint_group_velocity_controller/command std_msgs/Float64MultiArray "data: [0,0,0,0,0,0,-0.1,0,0,0,0,0,0,0.5]"
'''
import agx
import agxSDK
import agxOSG
import agxCollide
import agxRender
import agxModel
import agxPython
import agxIO
import sys
import os
import traceback
import utils 
import agxUtil


try:
    import rospy
    import rospkg
    from std_msgs.msg import Float64MultiArray
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import WrenchStamped
    from abb_egm_msgs.msg import EGMState, EGMChannelState
    from abb_rapid_sm_addin_msgs.srv import SetSGCommand
    from abb_robot_msgs.srv import TriggerWithResultCode

except Exception as e:
    print("ROS could not be imported.")
    traceback.print_exc()
    sys.exit(2)

from agxPythonModules.utils.environment import simulation, root, application, init_app

STEP_TIME = 1/120
CASE = 'grabDLO' # 'fixture', 'fixtureWire', 'grabWire' or 'grabDLO'

def rigidBodyGeometriesToList(rb):
    geometries = rb.getGeometries() # geometries is of type GeometryRefVector
    geometries_list = list()        # the list to return
    for geo_ref in geometries:      # geometries contain items of type GeometryRef
        geometries_list.append(geo_ref.get()) # geo_ref.get() gives us its Geometry
    return geometries_list            # geometries_list now contain Geometry items


def collisionBetweenBodies(RB1, RB2, collision=True):
    for i in range(len(RB1.getGeometries())):
        for j in range(len(RB2.getGeometries())):
            RB1.getGeometries()[i].setEnableCollisions(RB2.getGeometries()[j], collision)

# Class representing the yumi robot
class yumiRobot(agxSDK.StepEventListener):
    def __init__(self, yumi_assembly):
        super().__init__()
        
        # ROS subscriber and publisher
        self.vel_command_subscriber = rospy.Subscriber("/yumi/egm/joint_group_velocity_controller/command", Float64MultiArray, self.callback, queue_size=2, tcp_nodelay=True)
        self.pub = rospy.Publisher('/yumi/egm/joint_states', JointState, queue_size=2)
        self.pub_egm_state = rospy.Publisher('/yumi/egm/egm_states', EGMState, queue_size=2)
        self.pubForceTorque = rospy.Publisher('/Yumi/forceTorqueR', WrenchStamped, queue_size=1)
        # yumi  
        self.yumi = yumi_assembly

        # for reseting
        self._joints = []
        self._body_transformations = []

        # urdf name for joints 
        self.jointNamesRevolute = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l',\
                                 'yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r']
       
        self.jointEffort = [50,50,50,50,50,50,50,50,50,50,50,50,50,50] #maximum joint effort, assuming same force in upper and lower, same order as jointNamesRevolute
        self.grpperEffort  = 25 # set the grip force
        self.jointNamesGrippers = ['gripper_l_joint', 'gripper_l_joint_m', 'gripper_r_joint', 'gripper_r_joint_m'] # name of gripper joints in urdf
        self.gripperPosition = [0,0,0,0] # used to store gripper commands until they are used
        self.gripperPositionRun = [0,0,0,0] # uesd for controlling. Both are needed for emulating yumi behaviour 

        # initilizes
        self.init()

        # counter
        self.seq = 0


    def reset(self):
        print("Reset")
        for b, t in self._body_transformations:
            b.setTransform(t)
            b.setVelocity(agx.Vec3(0, 0, 0))
            b.setAngularVelocity(agx.Vec3(0, 0, 0))

        for c in self._joints:
            c.rebind()

    def init(self):

        # start subscriber to relevant ros topics and get callbacks when messages arrives

        rospy.Subscriber("/yumi/egm/joint_group_velocity_controller/command", Float64MultiArray, self.callback, queue_size=1)
        
        # create ros service for grippers
        rospy.Service('/yumi/rws/sm_addin/set_sg_command', SetSGCommand, self.receiveGripperCommand)
        rospy.Service('/yumi/rws/sm_addin/run_sg_routine', TriggerWithResultCode, self.setGripperCommand)

        # Enable Motor1D (speed controller) on all revolute joints and set effort limits 
        for i in range(len(self.jointNamesRevolute)):
            self.yumi.getConstraint1DOF(self.jointNamesRevolute[i]).getMotor1D().setEnable(True)
            self.yumi.getConstraint1DOF(self.jointNamesRevolute[i]).getMotor1D().setForceRange(-self.jointEffort[i], self.jointEffort[i])
        
        # Enable Motor1D (speed controller) on all prismatic joints (grippers) and set effort limits 
        for i in range(len(self.jointNamesGrippers)):
            self.yumi.getConstraint1DOF(self.jointNamesGrippers[i]).getMotor1D().setEnable(True)
            self.yumi.getConstraint1DOF(self.jointNamesGrippers[i]).getMotor1D().setForceRange(-self.grpperEffort, self.grpperEffort)

        for c in self.yumi.getConstraints():
            self._joints.append(c)

        for b in self.yumi.getRigidBodies():
            self._body_transformations.append([b, b.getTransform()])

        # Publish egem state as this part was used by another project. Not all topics that are published by the original YuMi diver is emulated
        self.msg_egm_state = EGMState()
        msg_channel0 = EGMChannelState()
        msg_channel1 = EGMChannelState()
        msg_channel0.active = True
        msg_channel1.active = True
        self.msg_egm_state.egm_channels = [msg_channel0, msg_channel1]


    def pre(self, time):
        # Originally used for ROS 2 ros spin, but not neccessary in ROS 1, 
        # used for publish angles as it is called every sim step and gripper control. 

        # simple p controller for the grippers 
        for i in range(len(self.jointNamesGrippers)):
            diff = self.gripperPositionRun[i] - self.yumi.getConstraint1DOF(self.jointNamesGrippers[i]).getAngle()
            self.yumi.getConstraint1DOF(self.jointNamesGrippers[i]).getMotor1D().setSpeed(diff)

        # publish joint positions 
        jointPositions = []
        jointVelocities = []
        for i in range(len(self.jointNamesRevolute)):
            jointPositions.append(self.yumi.getConstraint1DOF(self.jointNamesRevolute[i]).getAngle())
            jointVelocities.append(self.yumi.getConstraint1DOF(self.jointNamesRevolute[i]).getCurrentSpeed())

        # joint state message  
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self.seq
        self.seq += 1
        # naming after how ABB driver does it, not same as urdf file.  
        msg.name = ["yumi_robl_joint_1", "yumi_robl_joint_2", "yumi_robl_joint_3", \
                    "yumi_robl_joint_4", "yumi_robl_joint_5", "yumi_robl_joint_6",\
                    "yumi_robl_joint_7", "yumi_robr_joint_1", "yumi_robr_joint_2", "yumi_robr_joint_3", "yumi_robr_joint_4",\
                    "yumi_robr_joint_5", "yumi_robr_joint_6", "yumi_robr_joint_7"] 
        msg.position = jointPositions
        msg.velocity = jointVelocities

        # publish joint positions
        self.pub.publish(msg)
        # publish egm state
        self.pub_egm_state.publish(self.msg_egm_state)

        # meassure force and torque
        forces = [0,0,0,0,0,0]
        for i in range(6): 
            forces[i] = self.yumi.getConstraint('yumi_link_7_r_joint').getCurrentForce(i)
        
        # Publish force and torque 
        forceMsg = WrenchStamped()
        forceMsg.header.stamp = rospy.Time.now()
        forceMsg.header.seq = self.seq
        forceMsg.wrench.force.z = forces[0]
        forceMsg.wrench.force.y = forces[1]
        forceMsg.wrench.force.x = forces[2]
        forceMsg.wrench.torque.z = forces[3]
        forceMsg.wrench.torque.y = forces[4]
        forceMsg.wrench.torque.x = forces[5]
        self.pubForceTorque.publish(forceMsg)

    def receiveGripperCommand(self, SetSGCommand):
        # callback for gripper set_sg_command service, only 3 functionalities emulated, move to, grip in and grip out. 
        # indx for left gripper task
        if SetSGCommand.task == 'T_ROB_L':
            index_a = 0
            index_b = 1
        # inex for the right gripper 
        elif SetSGCommand.task == 'T_ROB_R':
            index_a = 2
            index_b = 3
        else: 
            return  [2, '']# returns failure state as service is finished

        if SetSGCommand.command == 5: # move to
            self.gripperPosition[index_a] = SetSGCommand.target_position /1000 # convert mm to meters
            self.gripperPosition[index_b] = SetSGCommand.target_position /1000 # convert mm to meters

        elif SetSGCommand.command == 6: # grip in
            self.gripperPosition[index_a] = 0
            self.gripperPosition[index_b] = 0     
        elif SetSGCommand.command == 7: # grip out
            self.gripperPosition[index_a] = 0.025 
            self.gripperPosition[index_b] = 0.025 
        else:
            return  [2, ''] # returns failure state as service is finished

        return [1, ''] # returns success state as service is finished

    def setGripperCommand(self, SetSGCommand):
        # callback for run_sg_routine, runs the gripper commands, i.e. grippers wont move before this service is called. 
        self.gripperPositionRun = self.gripperPosition.copy()
        return [1, '']

    def removeNotification(self):
        # Called at shutdown
        rospy.signal_shutdown('shutdown called')

    # callback for velocity commands. Sets new speed targets. 
    def callback(self, msg):
        for i in range(len(self.jointNamesRevolute)):
            self.yumi.getConstraint1DOF(self.jointNamesRevolute[i]).getMotor1D().setSpeed(msg.data[i])


def setupCamera(app):
    cameraData = app.getCameraData()
    cameraData.eye = agx.Vec3(0.3, -1, 0.1)
    cameraData.center = agx.Vec3(0.3, -0.0, 0.0)
    cameraData.up = agx.Vec3(-0.2857, -0.0514, 0.9569)
    cameraData.nearClippingPlane = 0.1
    cameraData.farClippingPlane = 5000
    app.applyCameraData(cameraData)


def buildScene():
    print('start build scene')

    app = agxPython.getContext().environment.getApplication()
    sim = agxPython.getContext().environment.getSimulation()
    root = agxPython.getContext().environment.getSceneRoot()
    # Change the timestep
    sim.setTimeStep(STEP_TIME)
    # Construct the floor that the yumi robot will stand on
    floor = agxCollide.Geometry(agxCollide.Box(agx.Vec3(1, 1, 0.05)))
    floor.setPosition(0, 0, -0.05)
    sim.add(floor)
    fl = agxOSG.createVisual(floor, root)
    agxOSG.setDiffuseColor(fl, agxRender.Color.LightGray())

    # Set the sky color
    application().getSceneDecorator().setBackgroundColor(agxRender.Color.SkyBlue() , agxRender.Color.DodgerBlue())

    # Read the URDF file
    rospack = rospkg.RosPack()
    pathToAGXPkg = rospack.get_path('agx_yumi_ros')
    urdf_file = pathToAGXPkg + '/yumi_description/urdf/yumi.urdf' 
    package_path = pathToAGXPkg

    # initial joint position 
    #initJointPosList = [1.0, -2.0, -1.2, 0.6, -2.0, 1.0, 0.0, 0.0, 0.0, -1.0, -2.0, 1.2, 0.6, 2.0, 1.0, 0.0, 0.0, 0.0]
    initJointPosList = [1.0157189974398513,  -1.2250581426342209, -1.0116368949128605, 1.3020200753993196, -2.0266021268656664, 1.5525773965548053, 1.3745989237267395,0,0,-1.0024065462675744, -1.2247615902383373, 1.0191185734510633, 1.2749531867574384, 2.0237406614692075, 1.56660964034699, -1.3754033952812534, 0, 0]
    initJointPos = agx.RealVector()
    for i in range(len(initJointPosList)):
        initJointPos.append(initJointPosList[i])
     
    # read urdf

    yumi_assembly_ref = agxModel.UrdfReader.read(urdf_file, package_path, initJointPos, True)
    if (yumi_assembly_ref.get() == None):
        print("Error reading the URDF file.")
        sys.exit(2)

    # Add the yumi assembly to the simulation and create visualization for it
    sim.add(yumi_assembly_ref.get())
    fl = agxOSG.createVisual(yumi_assembly_ref.get(), root)
    
    # Create the yumi robot representation with a ROS subscriber
    yumi = yumiRobot(yumi_assembly_ref.get())

    sim.add(yumi)

    # disable collision between floor and yumi body
    for i in range(len(yumi_assembly_ref.getRigidBody('yumi_body').getGeometries())):
        floor.setEnableCollisions(yumi_assembly_ref.getRigidBody('yumi_body').getGeometries()[i], False)

    # disable collision between connected links. 
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_body'), yumi_assembly_ref.getRigidBody('yumi_link_1_r'), False)
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_body'), yumi_assembly_ref.getRigidBody('yumi_link_1_l'), False)

    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_1_r'), yumi_assembly_ref.getRigidBody('yumi_link_2_r'), False)
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_1_l'), yumi_assembly_ref.getRigidBody('yumi_link_2_l'), False)

    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_2_r'), yumi_assembly_ref.getRigidBody('yumi_link_3_r'), False)
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_2_l'), yumi_assembly_ref.getRigidBody('yumi_link_3_l'), False)

    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_3_r'), yumi_assembly_ref.getRigidBody('yumi_link_4_r'), False)
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_3_l'), yumi_assembly_ref.getRigidBody('yumi_link_4_l'), False)

    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_4_r'), yumi_assembly_ref.getRigidBody('yumi_link_5_r'), False)
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_4_l'), yumi_assembly_ref.getRigidBody('yumi_link_5_l'), False)

    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_5_r'), yumi_assembly_ref.getRigidBody('yumi_link_6_r'), False)
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_5_l'), yumi_assembly_ref.getRigidBody('yumi_link_6_l'), False)

    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_6_r'), yumi_assembly_ref.getRigidBody('yumi_link_7_r'), False)
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_6_l'), yumi_assembly_ref.getRigidBody('yumi_link_7_l'), False)

    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_6_r'), yumi_assembly_ref.getRigidBody('yumi_link_7_r'), False)
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_6_l'), yumi_assembly_ref.getRigidBody('yumi_link_7_l'), False)

    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_7_r'), yumi_assembly_ref.getRigidBody('gripper_r_base'), False)
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('yumi_link_7_l'), yumi_assembly_ref.getRigidBody('gripper_l_base'), False)
    
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('gripper_r_base'), yumi_assembly_ref.getRigidBody('gripper_r_finger_r'), False)
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('gripper_r_base'), yumi_assembly_ref.getRigidBody('gripper_r_finger_l'), False)

    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('gripper_l_base'), yumi_assembly_ref.getRigidBody('gripper_l_finger_r'), False)
    collisionBetweenBodies(yumi_assembly_ref.getRigidBody('gripper_l_base'), yumi_assembly_ref.getRigidBody('gripper_l_finger_l'), False)
    
    # material 
    material_hard = agx.Material("Aluminum")
    l_l_body = yumi_assembly_ref.getRigidBody('gripper_l_finger_l')
    l_r_body = yumi_assembly_ref.getRigidBody('gripper_l_finger_r')
    r_l_body = yumi_assembly_ref.getRigidBody('gripper_r_finger_l')
    r_r_body = yumi_assembly_ref.getRigidBody('gripper_r_finger_r')
    use_capsules = True
    if use_capsules:
        utils.replace_collision_geometry_with_capsules(sim, l_l_body, material_hard)
        utils.replace_collision_geometry_with_capsules(sim, l_r_body, material_hard)
        utils.replace_collision_geometry_with_capsules(sim, r_l_body, material_hard)
        utils.replace_collision_geometry_with_capsules(sim, r_r_body, material_hard)
    else:
        yumi_assembly_ref.getRigidBody('gripper_l_finger_l').getGeometries()[1].setMaterial(material_hard)
        yumi_assembly_ref.getRigidBody('gripper_l_finger_r').getGeometries()[1].setMaterial(material_hard)
        yumi_assembly_ref.getRigidBody('gripper_r_finger_l').getGeometries()[1].setMaterial(material_hard)
        yumi_assembly_ref.getRigidBody('gripper_r_finger_r').getGeometries()[1].setMaterial(material_hard)

    if CASE == 'fixture':
        # DLO
        utils.create_DLO(sim, root, material_hard)
    
        # create fixture 
        rotation = agx.EulerAngles(0, 0, agx.degreesToRadians(90.0)) 
        utils.create_fixture(sim=sim, position=agx.Vec3(0.3,0,0), name='Fixture', root=root, rotation=rotation)

    elif CASE == 'fixtureWire':
        # DLO
        utils.createWire(sim, root, material_hard)
    
        # create fixture 
        rotation = agx.EulerAngles(0, 0, agx.degreesToRadians(90.0)) 
        utils.create_fixture(sim=sim, position=agx.Vec3(0.3,0,0), name='Fixture', root=root, rotation=rotation)

    elif CASE == 'grabDLO':
        utils.create_DLO_on_floor(sim, root, material_hard)
        rotation = agx.EulerAngles(0, 0, agx.degreesToRadians(90.0))
        utils.create_fixture(sim=sim, position=agx.Vec3(0.37, 0, 0), name='Fixture', root=root, rotation=rotation)
    elif CASE == 'grabWire':
        utils.create_wire_on_floor(sim, root, material_hard)
    else:
        pass
    # not working 
    #agxOSG.createAxes(yumi_assembly_ref.getConstraint('yumi_link_7_r_joint'), root, 1.0, agx.Vec4f(1,1,1,1))

    application().getSceneDecorator().setEnableShadows(True)
    setupCamera(app)
    return root

def init(app):
    # Initialize ROS
    agx.setNumThreads(8)
    rospy.init_node('AGX', anonymous=True) 


def shutdown(app):
    print("\nShutting down...")

    # Shut down ROS2
    rospy.signal_shutdown('shutdown called')

# Entry point when this script is started with python executable
init = init_app(name=__name__,
                scenes=[(buildScene, '1')],
                autoStepping=True,  # Default: False
                onInitialized=init,
                onShutdown=shutdown
                )

