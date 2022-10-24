#!/usr/bin/env python3

import numpy as np
import rospy
import agx
import agxIO
import agxUtil
import agxCollide
import agxOSG
import agxSDK
import agxCable 
import agxWire

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


def create_mesh(MESH_FILE, material, position, static, name, root, rotation):
    scaling = agx.Matrix3x3(agx.Vec3(1e-3))
    reader = agxIO.MeshReader()
    hullMesh = agxUtil.createTrimesh(MESH_FILE,0, scaling)
    hullGeom = agxCollide.Geometry(hullMesh, rotation) #not sure about rotation
    mesh = agx.RigidBody(name)
    mesh.add(hullGeom)

    if static == True:
        mesh.setMotionControl(agx.RigidBody.STATIC)
    else:
        mesh.setMotionControl(agx.RigidBody.DYNAMICS)

    hullGeom.setMaterial(material)
    mesh.setPosition(position)
    return mesh
    #sim.add(mesh)
    #fl = agxOSG.createVisual(mesh, root)


def create_fixture(sim, position, name, root, rotation):
    complience = 8e-1
    # Construct fixture
    assembly = agxSDK.Assembly()
    assembly.setName(name)
    assembly.setPosition(position)
    assembly.setRotation(rotation)
    
    position = agx.Vec3(0,0,0)
    rotation = agx.AffineMatrix4x4.rotate(agx.Vec3(0,0,0),agx.Vec3(0,0,0))
    material_hard = agx.Material("Aluminum1")

    # base
    MESH_FILE = '/home/gabwal/CatkinWorkspaces/catkin_yumi/src/agx_yumi_ros/yumi_description/SimMeshes/fixture_baseBase.obj'
    nameBase = name + 'base'
    baseMesh = create_mesh(MESH_FILE=MESH_FILE , material=material_hard, position=position, static=True, name=nameBase, root=root, rotation=rotation)
    assembly.add(baseMesh)

    # finger 1 
    MESH_FILE = '/home/gabwal/CatkinWorkspaces/catkin_yumi/src/agx_yumi_ros/yumi_description/SimMeshes/fixtureSimF.obj'
    nameFinger1 = name + 'finger1'
    rotation =  agx.AffineMatrix4x4.rotate(agx.Vec3(1,0,0),agx.Vec3(0,1,0))
    finger1Mesh = create_mesh(MESH_FILE=MESH_FILE , material=material_hard, position=position, static=False, name=nameFinger1, root=root, rotation=rotation)
    assembly.add(finger1Mesh)
    
    # hinge
    fixtureBase = assembly.getRigidBody(nameBase)
    fixtureFinger = assembly.getRigidBody(nameFinger1)
    frame1 = agx.Frame()
    frame2 = agx.Frame()
    frame1.setLocalTranslate( 0.007 ,0.003,0.015)
    frame1.setLocalRotate(agx.EulerAngles(0, agx.degreesToRadians(90.0),0) )
    frame2.setLocalTranslate( 0.0,0.003,0.015)
    frame2.setLocalRotate( agx.EulerAngles(0, agx.degreesToRadians(90.0),0) )
    hinge1 = agx.Hinge(fixtureBase, frame1, fixtureFinger, frame2)
    hinge1.getLock1D().setEnable(True)
    hinge1.getLock1D().setPosition(0)
    hinge1.getLock1D().setCompliance(complience )
    assembly.add(hinge1)

    # finger 2
    MESH_FILE = '/home/gabwal/CatkinWorkspaces/catkin_yumi/src/agx_yumi_ros/yumi_description/SimMeshes/fixtureSimF.obj'
    nameFinger2 =  name + 'finger2' 
    rotation =  agx.AffineMatrix4x4.rotate(agx.Vec3(1,0,0),agx.Vec3(0,1,0))
    finger2Mesh = create_mesh(MESH_FILE=MESH_FILE , material=material_hard, position=position, static=False, name=nameFinger2, root=root, rotation=rotation)
    assembly.add(finger2Mesh)

    # hinge 2
    fixtureBase = assembly.getRigidBody(nameBase)
    fixtureFinger = assembly.getRigidBody(nameFinger2)
    frame11 = agx.Frame()
    frame21 = agx.Frame()
    frame11.setLocalTranslate( -0.007 ,-0.003,0.015)
    frame11.setLocalRotate(agx.EulerAngles(0, agx.degreesToRadians(90.0),0) )
    frame21.setLocalTranslate( 0.0,0.003,0.015)
    frame21.setLocalRotate( agx.EulerAngles(0, agx.degreesToRadians(90.0),agx.degreesToRadians(180.0)) )
    hinge2 = agx.Hinge(fixtureBase, frame11, fixtureFinger, frame21)
    hinge2.getLock1D().setEnable(True)
    hinge2.getLock1D().setEnable(True)
    hinge2.getLock1D().setPosition(0)
    hinge2.getLock1D().setCompliance( complience )
    assembly.add(hinge2)

    # add to sim and make vissible 
    sim.add(assembly)
    fl = agxOSG.createVisual(assembly, root)



def create_DLO(sim, root, material_hard):


    # Rope parameters
    radious = 0.005  # meters
    resolution = 100  # segments per meter
    pegPoissionRatio = 0.1  # no unit
    youngMoculusBend = 2e5  # 1e4
    YoungModulusTwist = 2e5  # 1e10
    YoungModulusStretch = 1e7  # Pascals
    dampening = 1e4
    # Create rope and set name + properties
    peg = agxCable.Cable(radious,resolution)
    peg.setName("DLO")
    material_peg = peg.getMaterial()
    peg_material = material_peg.getBulkMaterial()
    peg_material.setPoissonsRatio(pegPoissionRatio)
    properties = peg.getCableProperties()
    properties.setYoungsModulus(youngMoculusBend, agxCable.BEND)
    properties.setYoungsModulus(YoungModulusTwist, agxCable.TWIST)
    properties.setYoungsModulus(YoungModulusStretch, agxCable.STRETCH)

    # Add connection between cable and gripper
    tf_0 = agx.AffineMatrix4x4()
    tf_0.setTranslate(0.0 ,0, 0.135)
    tf_0.setRotate(agx.EulerAngles(agx.degreesToRadians(-90.0),0, 0) )

    peg.add(agxCable.BodyFixedNode(sim.getRigidBody("gripper_r_base"), tf_0))

    freePos = sim.getRigidBody("gripper_r_base").getTransform().transformPoint(agx.Vec3(0.0, 0.1, 0.15))
    peg.add(agxCable.FreeNode(freePos))

    tf_0 = agx.AffineMatrix4x4()
    tf_0.setTranslate(0.0 ,0, 0.135)
    tf_0.setRotate(agx.EulerAngles(agx.degreesToRadians(-90.0),0, 0) )
    peg.add(agxCable.BodyFixedNode(sim.getRigidBody("gripper_l_base"), tf_0))


    sim.add(peg)
    
    segment_iterator = peg.begin()
    n_segments = peg.getNumSegments()
    for i in range(n_segments):
        if not segment_iterator.isEnd():
            seg = segment_iterator.getRigidBody()
            seg.setAngularVelocityDamping(dampening)
            segment_iterator.inc()
    
    # Try to initialize rope
    report = peg.tryInitialize()
    if report.successful():
        print("Successful rope initialization.")
    else:
        print(report.getActualError())
    
    # Add rope to simulation
    #sim.add(peg)
    

    # Set rope material
    material_peg = peg.getMaterial()
    material_peg.setName("rope_material")
    

    contactMaterial = sim.getMaterialManager().getOrCreateContactMaterial(material_hard, material_peg)
    contactMaterial.setYoungsModulus(1e12)
    fm = agx.IterativeProjectedConeFriction()
    fm.setSolveType(agx.FrictionModel.DIRECT)
    contactMaterial.setFrictionModel(fm)
    
    rbs = peg.getRigidBodies()
    for i in range(len(rbs)):
        rbs[i].setName('dlo_' + str(i+1))


    dlo = agxCable.Cable.find(sim, "DLO")
    fl = agxOSG.createVisual(dlo, root)

    agxUtil.setEnableCollisions(dlo,  sim.getAssembly('yumi').getRigidBody('gripper_r_finger_r'), False)
    agxUtil.setEnableCollisions(dlo,  sim.getAssembly('yumi').getRigidBody('gripper_r_finger_l'), False)
    agxUtil.setEnableCollisions(dlo,  sim.getAssembly('yumi').getRigidBody('gripper_l_finger_r'), False)
    agxUtil.setEnableCollisions(dlo,  sim.getAssembly('yumi').getRigidBody('gripper_l_finger_l'), False)


def createWire(sim, root, material_hard):
    wireMaterial = agx.Material("WireMaterial")
    wireMaterial.getWireMaterial().setYoungsModulusStretch(200E9)

    wireRadius = 0.005
    wireResolutionPerUnitLength = 1  # Lumped elements per meter.
    wireLength = 100

    # Create the wire.
    wire = agxWire.Wire(wireRadius, wireResolutionPerUnitLength)

    # Add a renderer for the wire
    sim.add(agxOSG.WireRenderer(wire, root))

    wire.setMaterial(wireMaterial)

    # Add connection between cable and gripper
    tf_0 = agx.AffineMatrix4x4()
    tf_0.setTranslate(0.0 ,0, 0.135)
    tf_0.setRotate(agx.EulerAngles(agx.degreesToRadians(-90.0),0, 0) )

    wire.add(agxWire.BodyFixedNode(sim.getRigidBody("gripper_r_base"), agx.Vec3(0, 0, 0.135)))

    tf_0 = agx.AffineMatrix4x4()
    tf_0.setTranslate(0.0 ,0, 0.135)
    tf_0.setRotate(agx.EulerAngles(agx.degreesToRadians(-90.0),0, 0) )
    wire.add(agxWire.BodyFixedNode(sim.getRigidBody("gripper_l_base"), agx.Vec3(0, 0, 0.135)))

    # Give the bodies in the wire a lot of velocity damping so it does not vibrate
    wire.setLinearVelocityDamping(1)
    sim.add(wire)

    #agxUtil.setEnableCollisions(wire,  sim.getAssembly('yumi').getRigidBody('gripper_r_finger_r'), False)
    #agxUtil.setEnableCollisions(wire,  sim.getAssembly('yumi').getRigidBody('gripper_r_finger_l'), False)
    #agxUtil.setEnableCollisions(wire,  sim.getAssembly('yumi').getRigidBody('gripper_l_finger_r'), False)
    #agxUtil.setEnableCollisions(wire,  sim.getAssembly('yumi').getRigidBody('gripper_l_finger_l'), False)



def create_DLO_on_floor(sim, root, material_hard):

    # Rope parameters
    radious = 0.005  # meters
    resolution = 100  # segments per meter
    pegPoissionRatio = 0.1  # no unit
    youngMoculusBend = 2e5  # 1e4
    YoungModulusTwist = 2e5  # 1e10
    YoungModulusStretch = 1e8  # Pascals
    dampening = 1e4
    # Create rope and set name + properties
    peg = agxCable.Cable(radious,resolution)
    peg.setName("DLO")
    material_peg = peg.getMaterial()
    peg_material = material_peg.getBulkMaterial()
    peg_material.setPoissonsRatio(pegPoissionRatio)
    properties = peg.getCableProperties()
    properties.setYoungsModulus(youngMoculusBend, agxCable.BEND)
    properties.setYoungsModulus(YoungModulusTwist, agxCable.TWIST)
    properties.setYoungsModulus(YoungModulusStretch, agxCable.STRETCH)

    
    peg.add(agxCable.FreeNode(agx.Vec3(0.3, -0.3, 0.01)))
    peg.add(agxCable.FreeNode(agx.Vec3(0.3, 0.3, 0.01)))

    sim.add(peg)
    
    segment_iterator = peg.begin()
    n_segments = peg.getNumSegments()
    for i in range(n_segments):
        if not segment_iterator.isEnd():
            seg = segment_iterator.getRigidBody()
            seg.setAngularVelocityDamping(dampening)
            segment_iterator.inc()
    
    # Try to initialize rope
    report = peg.tryInitialize()
    if report.successful():
        print("Successful rope initialization.")
    else:
        print(report.getActualError())

    # Set rope material
    material_peg = peg.getMaterial()
    material_peg.setName("rope_material")

    contactMaterial = sim.getMaterialManager().getOrCreateContactMaterial(material_hard, material_peg)
    contactMaterial.setYoungsModulus(1e12)
    #contactMaterial.setAdhesion(0.5, 0.0001)
    fm = agx.IterativeProjectedConeFriction()
    fm.setSolveType(agx.FrictionModel.DIRECT)
    contactMaterial.setFrictionModel(fm)

    contactMaterial.setUseContactAreaApproach(True)
    sim.add(contactMaterial)
    rbs = peg.getRigidBodies()
    for i in range(len(rbs)):
        rbs[i].setName('dlo_' + str(i+1))

    dlo = agxCable.Cable.find(sim, "DLO")
    fl = agxOSG.createVisual(dlo, root)


def create_wire_on_floor(sim, root, material_hard):

    wireMaterial = agx.Material("WireMaterial")
    wireMaterial.getWireMaterial().setYoungsModulusStretch(200E9)

    wireRadius = 0.005
    wireResolutionPerUnitLength = 1  # Lumped elements per meter.
    wireLength = 100

    # Create the wire.
    wire = agxWire.Wire(wireRadius, wireResolutionPerUnitLength)

    # Add a renderer for the wire
    sim.add(agxOSG.WireRenderer(wire, root))

    wire.setMaterial(wireMaterial)

    node = agxWire.FreeNode(agx.Vec3(0.3, -0.3, 0.01))
    wire.add(node)
    node = agxWire.FreeNode(agx.Vec3(0.3, 0.3, 0.01))
    wire.add(node)
    # Give the bodies in the wire a lot of velocity damping so it does not vibrate
    wire.setLinearVelocityDamping(1)
    sim.add(wire)


def create_capsule(sim, material, radius):
    c = agxCollide.Geometry(agxCollide.Capsule(radius, 0.038))
    c.setMaterial(material)
    sim.add(c)
    return c


def replace_collision_geometry_with_capsules(sim, body, material):
    # Disable the other collision geometries
    for g in body.getGeometries():
        g.setEnableCollisions(False)
    capsule_radius = 0.001
    c_l = create_capsule(sim, material, capsule_radius)
    c_r = create_capsule(sim, material, capsule_radius)
    o_l = agx.Vec3(-capsule_radius, 0.032, -0.003)
    o_r = agx.Vec3(-capsule_radius, 0.032, -0.010)

    t_l = agx.AffineMatrix4x4.translate(o_l) * agx.AffineMatrix4x4.rotate(agx.PI_2, agx.Vec3(1, 0, 0))
    t_r = agx.AffineMatrix4x4.translate(o_r) * agx.AffineMatrix4x4.rotate(agx.PI_2, agx.Vec3(1, 0, 0))

    body.add(c_l, t_l)
    body.add(c_r, t_r)

