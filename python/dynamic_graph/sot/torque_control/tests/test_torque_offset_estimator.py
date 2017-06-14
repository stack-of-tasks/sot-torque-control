# -*- coding: utf-8 -*-
"""
2017, LAAS/CNRS
@author: Rohan Budhiraja
"""



# ______________________________________________________________________________
# ******************************************************************************
#
#1)  The simplest robot task: Just go and reach a point
#2)  While performing this task:
# )               find the torque values and feed them to the sensor calibration entity.
#3)  Confirm that the offsets being calculated are correct.
# ______________________________________________________________________________
# ******************************************************************************

# Requires sot-dynamic-pinocchio

#-----------------------------------------------------------------------------


#SET THE PATH TO THE URDF AND MESHES
#Define robotName, urdfpath and initialConfig
dt = 5e-3
urdfPath = "/local/rbudhira/git/pyrene/talos-data/robots/talos_reduced.urdf"
urdfDir = ["/local/rbudhira/git/pyrene/talos-data"]
robotName = 'TALOS'
OperationalPointsMap = {'left-wrist'  : 'arm_left_7_joint',
                         'right-wrist' : 'arm_right_7_joint',
                         'left-ankle'  : 'leg_left_5_joint',
                         'right-ankle' : 'leg_right_5_joint',
                         'gaze'        : 'head_2_joint',
                         'waist'       : 'root_joint',
                         'chest'       : 'torso_2_joint'}
initialConfig = (0., 0., 0.648702, 0., 0. , 0.,                  # Free flyer
                 0., 0., -0.453786, 0.872665, -0.418879, 0.,     # Left Leg
                 0., 0., -0.453786, 0.872665, -0.418879, 0.,     # Right Leg
                 0., 0.,                                         # Chest
                 0.261799, 0.17453, 0., -0.523599, 0., 0., 0.1,0.0,  # Left Arm
                 #0., 0.,0.,0., 0.,0.,0.,                         # Left gripper
                 -0.261799, -0.17453, 0., -0.523599, 0., 0., 0.1,0.0,  # Right Arm
                 #0., 0.,0.,0., 0.,0.,0.,                         # Right gripper
                 0., 0.                                          # Head
             )


#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------
from pinocchio.robot_wrapper import RobotWrapper
import pinocchio as se3
from dynamic_graph.sot.dynamics_pinocchio import fromSotToPinocchio
pinocchioRobot = RobotWrapper(urdfPath, urdfDir, se3.JointModelFreeFlyer())
pinocchioRobot.initDisplay(loadModel=True)
pinocchioRobot.display(fromSotToPinocchio(initialConfig))


from dynamic_graph.sot.dynamics_pinocchio.humanoid_robot import HumanoidRobot
robot = HumanoidRobot(robotName, pinocchioRobot.model,
                      pinocchioRobot.data, initialConfig, OperationalPointsMap)


# ------------------------------------------------------------------------------
# ---- Kinematic Stack of Tasks (SoT)  -----------------------------------------
# ------------------------------------------------------------------------------
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
sot = SOT('sot')
sot.setSize(robot.dynamic.getDimension())
plug(sot.control,robot.device.control)

# ------------------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------

# ---- TASK GRIP
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from numpy import eye

taskRH    = MetaTaskKine6d('rh',robot.dynamic,'rh',robot.OperationalPointsMap['right-wrist'])
handMgrip = eye(4); handMgrip[0:3,3] = (0.1,0,0)
taskRH.opmodif = matrixToTuple(handMgrip)
taskRH.feature.frame('desired')

taskLH    = MetaTaskKine6d('lh',robot.dynamic,'lh',robot.OperationalPointsMap['left-wrist'])
taskLH.opmodif = matrixToTuple(handMgrip)
taskLH.feature.frame('desired')

# --- STATIC COM (if not walking)
taskCom = MetaTaskKineCom(robot.dynamic)
robot.dynamic.com.recompute(0)
taskCom.featureDes.errorIN.value = robot.dynamic.com.value
taskCom.task.controlGain.value = 10


# --- CONTACTS
#define contactLF and contactRF
for name,joint in [ ['LF',robot.OperationalPointsMap['left-ankle']], ['RF',robot.OperationalPointsMap['right-ankle'] ] ]:
    contact = MetaTaskKine6d('contact'+name,robot.dynamic,name,joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    contact.keep()
    locals()['contact'+name] = contact


targetRH = (0.5,-0.2,1.0)
targetLH = (0.5,0.2,1.0)

#addRobotViewer(robot, small=False)
#robot.viewer.updateElementConfig('zmp',target+(0,0,0))

gotoNd(taskRH,targetRH,'111',(4.9,0.9,0.01,0.9))
gotoNd(taskLH,targetLH,'111',(4.9,0.9,0.01,0.9))
sot.push(contactRF.task.name)
sot.push(contactLF.task.name)
sot.push(taskCom.task.name)
sot.push(taskLH.task.name)
sot.push(taskRH.task.name)


# Sensor Calibration Entity.
from dynamic_graph.sot.torque_control.torque_offset_estimator import TorqueOffsetEstimator as TOE
import numpy as np

toe = TOE("test_entity_toe")
id4 = ((1.0,0.0,0.0,0.0),(0.0,1.0,0.0,0.0),(0.0,0.0,1.0,0.0),(0.0,0.0,0.0,1.0))
toe.init(urdfPath, id4)
plug(robot.device.state, toe.base6d_encoders)

from pinocchio import Motion
gravity = Motion.Zero()
gravity.linear = np.asarray((0.0,0.0,-9.81))
#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
robot.device.increment(dt)
from pinocchio import rnea

np.set_printoptions(suppress=True)

def runner(n):
    for i in xrange(n):
        q_pin = fromSotToPinocchio(robot.device.state.value)
        nvZero = np.asarray((0.0,)*pinocchioRobot.model.nv)
        tau = np.asarray(rnea(pinocchioRobot.model, pinocchioRobot.data, q_pin, nvZero, nvZero)).squeeze()
        tau_offset = np.asarray((0.1,)*tau.size).squeeze()
        toe.jointTorques.value = tau_offset + tau

        toe.accelerometer.value = np.asarray((pinocchioRobot.data.oMi[15].inverse()*pinocchioRobot.data.oMi[1]*gravity).linear).squeeze()

        toe.jointTorquesEstimated.recompute(robot.device.state.time)
        #print toe.jointTorques.value
        entity_offset = np.asarray(toe.jointTorquesEstimated.value).squeeze() - np.asarray(toe.jointTorques.value).squeeze()

        robot.device.increment(dt)
        pinocchioRobot.display(q_pin)


runner(10)
toe.computeOffset(100,1.0)
runner(100)
runner(1000)

print toe.getSensorOffsets()
