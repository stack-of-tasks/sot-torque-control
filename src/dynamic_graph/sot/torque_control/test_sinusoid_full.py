# This uses run_command assuming a prolog

from dynamic_graph import plug
from dynamic_graph.sot.torque_control import HRP2DevicePosCtrl, TestSinusoidControl


def initialize(robot):
    halfSitting = (0, 0, -0.4538, 0.8727, -0.4189, 0,
                   0, 0, -0.4538, 0.8727, -0.4189, 0,
                   0, 0, 0, 0,
                   0.2618, -0.1745, 0, -0.5236, 0, 0, 0,
                   0.2618,  0.1745, 0, -0.5236, 0, 0, 0)

    device_position = HRP2DevicePosCtrl("hrp2")
    nj = 30
    device_position.resize(nj)
    device_position.set(halfSitting)
    device_position.kp.value = nj * (1, )
    device_position.kd.value = nj * (2 * pow(device_position.kp.value[0], 0.5), )
    robot.device = device_position

    control = TestSinusoidControl('control')
    control.setAmplitude(0.5)
    control.setFrequency(0.1)
    control.setJointID(3)
    control.dt.value = robot.timeStep
    robot.sinControl = control

    plug(robot.device.state, control.positionMeas)
    plug(control.positionDes, robot.device.control)
