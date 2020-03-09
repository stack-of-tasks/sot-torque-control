from dynamic_graph import plug
from dynamic_graph.sot.torque_control import HRP2DevicePosCtrl, TestSinusoidControl
from dynamic_graph.tracer import Tracer

# Initializations
# initialConfig = (0,0,0.648697398115,0,0,0) +
halfSitting = (0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, 0, 0, 0.2618, -0.1745, 0,
               -0.5236, 0, 0, 0, 0.2618, 0.1745, 0, -0.5236, 0, 0, 0)
dt = 0.001

# Create an instance of the device
device = HRP2DevicePosCtrl("hrp2")
nj = 30
device.resize(nj)
device.set(halfSitting)
device.kp.value = nj * (1, )
device.kd.value = nj * (2 * pow(device.kp.value[0], 0.5), )

# Create an instance to send a sinusoid
control = TestSinusoidControl('control')
control.setAmplitude(0.5)
control.setFrequency(0.1)
control.setJointID(3)
control.dt.value = dt

# Plug device and control
plug(device.state, control.positionMeas)
plug(control.positionDes, device.control)

# Increment the device
device.increment(dt)

# Start the control
control.start()

tr = Tracer('tr')
tr.open('/tmp/', 'dyn_', '.dat')

tr.add('control.positionDes', '')
tr.start()
device.after.addSignal('tr.triger')

for i in range(1000):
    device.increment(dt)

print("finished ...")
