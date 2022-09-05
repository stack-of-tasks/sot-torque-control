# I’ve thought a bit about how to identify the cause of the vibrations we observe on the robot. I came up with this series of experiments that we could do tomorrow.
# You can keep working on the base estimator in the mean time, but I’d like to know what you think about this plan.
# The key idea is to remove all potential sources of noise, and then reintroduce them one by one, until we don’t see the vibrations appearing again.
# As a starting point, we should remove all filters, or replace them with very soft ones, doing something like this:

# No filtering at all
b = (1.0, 0.0)
a = (1.0, 0.0)

# Filter with smallest delay among the ones found by Rohand
b = (0.00902094, 0.01800633, 0.00902094)
a = (1.0, -1.72610371, 0.76236003)

# Butter Wn=0.05, delay=9*dt
b = (0.00554272, 0.01108543, 0.00554272)
a = (1.0, -1.77863178, 0.80080265)

robot.filters.ft_RF_filter.switch_filter(b, a)
robot.filters.ft_LF_filter.switch_filter(b, a)
robot.filters.ft_RH_filter.switch_filter(b, a)
robot.filters.ft_LH_filter.switch_filter(b, a)
robot.filters.acc_filter.switch_filter(b, a)
# robot.filters.gyro_filter.switch_filter(b, a)
# robot.filters.estimator_kin.switch_filter(b, a)

b = (2.16439898e-05, 4.43473520e-05, -1.74065002e-05, -8.02197247e-05, -1.74065002e-05, 4.43473520e-05, 2.16439898e-05)
a = (1.0, -5.32595322, 11.89749109, -14.26803139, 9.68705647, -3.52968633, 0.53914042)
robot.filters.gyro_filter.switch_filter(b, a)
robot.filters.estimator_kin.switch_filter(b, a)

K = (1e10, 1e10, 1e10, 707, 502, 1e10)
robot.base_estimator.set_stiffness_left_foot(K)
robot.base_estimator.set_stiffness_right_foot(K)

robot.inv_dyn.w_forces.value = 1e-4
smoothly_set_signal(robot.inv_dyn.kp_com, (20.0, 20.0, 20.0))
smoothly_set_signal(robot.inv_dyn.kd_com, (7.0, 0.0, 0.0))

kp_torque = list(robot.torque_ctrl.KpTorque.value)
kp_torque[4] = 0.5
kp_torque[4 + 6] = 0.5
smoothly_set_signal(robot.torque_ctrl.KpTorque, tuple(kp_torque))

# Then we can proceed with the following tests.

# !!! Test 1: Baseline
# RESULTS: robot is rather stable, only a bit of jerky motions coming from left knee,
# but not amplified by control.
"""
* set imu weight to 0 in base estimator
* set w-com, kp-com and kd-com to 0 in balance controller
* set kp-torque to 0 in torque controller
* unplug accelerometer and joint vel from torque estimator
"""

# robot.estimator_ft.accelerometer.value = robot.estimator_ft.accelerometer.value
# robot.estimator_ft.dq_filtered.value = 30*(0.0,)
robot.base_estimator.set_imu_weight(0.0)
# robot.inv_dyn.w_com.value = 0
# smoothly_set_signal(robot.inv_dyn.kp_com, 3*(0.,))
# smoothly_set_signal(robot.inv_dyn.kd_com, 3*(0.,))
smoothly_set_signal(robot.torque_ctrl.KpTorque, 30 * (0.0,))

# After doing that, the system should be stable because the only feedback comes from the
# encoders (which are basically noiseless), the current sensors (which are only used for
# the dead-zone compensation and the integral feedback) and the joint velocities used
# for friction and back-EMF compensation (which did not give any vibration with the
# robot in the air).
# If we still have vibrations with this setup it could potentially be good news because
# it would mean we can remove them easily, filtering more the current sensors or the
# joint velocities, but I doubt that'll be the case.

#!!! Test 2: FT sensors through Joint Torques
# RESULTS: Vibrations clearly visible as soon as you move the robot
"""
* leave imu weight to 0 in base estimator
* leave w-com, kp-com and kd-com to 0 in balance controller
* increase kp-torque in torque controller
"""

# This is my main suspect as source of vibrations. However, in earlier experiments we've
# seen that for sure this is not the only source, because we experienced vibrations even
# when kp-torque was set to zero.

#!!! Test 3: FT sensors through Base Pose
# RESULTS: Robot stable, but tracking very poor even with kp_com=40
"""
* leave imu weight to 0 in base estimator
* set kp-torque to 0 in torque controller
* set w-com to 1, and increase kp-com in balance controller
"""

# This test investigates the noise of the base estimation. Ithis noise turns out to
# critical, we can always reduce it by neglecting certain axes of the ft measurements.
# While we know that the flexibility allows for 6d deformations, we also know that the
# main deflections happen in roll and pitch, so we could neglect the other directions,
# thus reducing the noise in the base estimation. If the noise really comes from the
# roll-pitch measurements, then the only option is to filter more.

#!!! Test 3B: Add Torque Integral
# RESULTS: No vibrations, better tracking, but controlled divergence of CoM. We noticed
# an offset of about 1 cm in CoM w.r.t. CoP, which may contribute to this effect, so we
# tried to compensate for it in balance controller. It improved a bit the behavior, but
# it was still diverging, which is not surprising given the lack of damping. Setting
# kp-torque to 2 for the ankle pitch joints improved the CoP tracking, but the
# divergence didn't disappear.

# !!! Test 4: IMU through Base Pose
"""
* leave kp-torque to 0 in torque controller
* leave w-com, kp-com and kd-com in balance controller
* set imu weight to 1 in base estimator
"""

# This is just to see whether the IMU introduces significant noise in the base
# estimation. If it does, we can simply stop using it, or try to filter it more.

# !!! Test 5: Gyro\Encoders through Base Velocity
"""
* set imu weight to 0 in base estimator
* leave kp-torque to 0 in torque controller
* set kp-com to 0 and increase kd-com  in balance controller
"""

# This is just to test the base velocity estimation. If we see vibrations, we can then
# investigate whether they come form the gyroscope of the encoders. In any case, more
# filtering is the only solution I see in this case.
