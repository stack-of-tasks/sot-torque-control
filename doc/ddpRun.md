# Running the DDP on the right elbow joint of Pyrene

In the following, we demonstrate how to run a test with  sot-torque-control, <a href="https://github.com/stack-of-tasks/ddp-actuator-solver">ddp-actuator-solver</a> and <a href="https://gitlab.laas.fr/pyrene-dev/talos-torque-control.git">talos-torque-control</a>.

## Start the simulation

Start the simulation with the robot in the half-sitting position:
```
roslaunch talos_data talos_gazebo.launch start_half_sitting:=true
```
If you are on the real robot just ignore this step.

## Start the SoT in torque mode

To start the SoT in simulation in torque mode:
```
roslaunch roscontrol_sot_talos sot_talos_controller_gazebo_effort.launch
```

To start the SoT on the real robot in torque mode:
```
roslaunch roscontrol_sot_talos sot_talos_controller_effort.launch
```

## Run the test

First of all, you need to go to the folder where your script is.
For running the ddp test of talos-torque-control, assuming you are in the root directory:

```
cd script
```

Then, you can just run the test, specifying that you are in simulation mode:

```
python test_ddp_sinu_effort.py simu
```

Or on the real robot:

```
python test_ddp_sinu_effort.py robot
```

This will launch the test making the robot executing a sinusoid with its right arm (on the elbow joint) using the DDP solver.
This one computes a trajectory avoiding the limits in position, velocity and torque of the robot.

The script also saves the dynamic graph in /tmp/sot_ddp_talos_effort.pdf.

You can add loads on the arm of the robot and see how it will react: by degrading the sinusoid movement of its elbow to respect its limits.

To make the loads taken into account by the DDP formulation you should use the following commands:

```
rosrun dynamic_graph_bridge run_command
robot.ddp_ctrl.setLoadParam(30.0,-0.021481595, 0.10)
```

It will add a load of 30kg at the coordinates (-0.021481595, 0.10) with respect to the elbow joint (10cm ahead on the arm from the joint, almost centered).

To remove the load just run:

```
robot.ddp_ctrl.removeLoad()
```

## Other

More information on how to use the SoT and how to work on Talos can be found <a href="https://wiki.laas.fr/robots/Pyrene">in the robot wiki page</a> (you need LAAS permissions to access this).
