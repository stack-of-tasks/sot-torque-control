# Pyrene CoM sinusoid in position or torque control

In the following, we quickly demonstrate how to run a test with sot-torque-control and <a href="https://gitlab.laas.fr/pyrene-dev/talos-torque-control.git">talos-torque-control</a>.

## Start the simulation

First of all, you need to start the simulation.

To start Gazebo, load a scene and spawn Talos, the simplest way is to directly use the launch file provided by PAL:
```
roslaunch talos_gazebo talos_gazebo.launch
```

Notice that this will spawn Talos at configuration zero. This is not always what you want.
The package talos_data offers different launch files to spawn it at different configurations.

Most commonly, you might want to spawn the robot in the half-sitting position:
```
roslaunch talos_data talos_gazebo.launch start_half_sitting:=true
```


## Start the SoT in position mode

To start the SoT in simulation in position mode:
```
roslaunch roscontrol_sot_talos sot_talos_controller_gazebo.launch
```

## Start the SoT in torque mode

To start the SoT in simulation in torque mode:
```
roslaunch roscontrol_sot_talos sot_talos_controller_gazebo_effort.launch
```

## Run the test

First of all, you need to go to the folder where your script is.
For instance, for running the standard tests of talos-torque-control, assuming you are in the root directory:

```
cd script
```

Then, you can just run the chosen test. For instance:

```
python sim_com.py position
```

This will launch a test routine executing a sinusoid on the CoM of the robot in position control (you should have started the SoT in position mode).
It also saves the dynamic graph in /tmp/sot_talos_tsid_com.pdf.

### Graph of the architecture

\image html Sot_torque_control_framework_for_position_control.png


For torque control simply run:

```
python sim_com.py torque
```

### Graph of the architecture

\image html Sot_torque_control_framework_for_torque_control.png

These two examples replicate the ex2 of the TSID jupyter notebooks in position and torque control in Gazebo (see the jupyter <a href="https://github.com/stack-of-tasks/tsid/blob/master/exercizes/notebooks/TSID_ex2.ipynb">here</a>).


## Interacting with the dynamic graph

If you want to dynamically interact with the graph

```
rosrun dynamic_graph_bridge run_command
```

## Other

More information on how to use the SoT and how to work on Talos can be found <a href="https://wiki.laas.fr/robots/Pyrene">in the robot wiki page</a> (you need LAAS permissions to access this).
