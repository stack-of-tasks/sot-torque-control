from dynamic_graph.sot.torque_control.main import *
from dynamic_graph import plug
robot.timeStep=0.001
robot = main_v3(robot, startSoT=True, go_half_sitting=True)
robot.ctrl_manager.ctrl_torque.value = 30*(0.,)
robot.ctrl_manager.ctrl_max.value = 30*(20.,)
create_topic(robot.ros, robot.device.currents,                    'i');
create_topic(robot.ros, robot.ctrl_manager.currents_real,         'i_real');
create_topic(robot.ros, robot.ctrl_manager.pwmDes,                'i_des')
create_topic(robot.ros, robot.device.robotState,                  'robotState')
create_topic(robot.ros, robot.ctrl_manager.pwmDesSafe,            'ctrl')
create_topic(robot.ros, robot.ctrl_manager.current_errors,        'i_err');
create_topic(robot.ros, robot.estimator_ft.jointsTorques,       'tau');
create_topic(robot.ros, robot.torque_ctrl.jointsTorquesDesired, 'tau_des');

# identification
from dynamic_graph.sot.torque_control.identification.identification_utils import *
from dynamic_graph.sot.torque_control.create_entities_utils import *
tracer=create_tracer(robot.device)

tracer.start();
identify_lhy_static(robot.traj_gen)
tracer.stop();

robot.traj_gen.moveJoint('lsp',-1.57,5.0)
robot.traj_gen.moveJoint('rsp',-1.57,5.0)
robot.traj_gen.moveJoint('le',-1.57,5.0)
robot.traj_gen.moveJoint('re',-1.57,5.0)
robot.traj_gen.moveJoint('lhr',+0.3,2.0)
robot.traj_gen.moveJoint('rhr',-0.3,2.0)
tracer.start();
tracer.stop();

robot.traj_gen.moveJoint('lhp',-1.5, 3)

tracer.start();
identify_lhy_dynamic(robot.traj_gen,'constVel',N=3,times=[5.0,2.5,1.5,1.0])
tracer.stop();

tracer.start();
identify_lhr_dynamic(robot.traj_gen,'constVel',N=3,times=[4.0,2.0,1.5,1.0])
tracer.stop();

tracer.start();
identify_lhp_dynamic(robot.traj_gen,'constVel',N=3,times=[4.0,2.0,1.5])
tracer.stop();


