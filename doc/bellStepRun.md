# Pyrene step in the air in torque control

In the following, we demonstrate how to run the foot sinusoid simulation with  <a href="https://github.com/stack-of-tasks/sot-torque-control">sot-torque-control</a>, and talos-torque-control.

## Start the simulation

Start the simulation with the robot in the half-sitting position:
```
roslaunch talos_data talos_gazebo.launch start_half_sitting:=true
```

## Start the SoT in torque mode

To start the SoT in simulation in torque mode: 
```
roslaunch roscontrol_sot_talos sot_talos_controller_gazebo_effort.launch
```

## Run the test

First of all, you need to go to the folder where your script is.
For running the ddp test of talos-torque-control, assuming you are in the root directory:

```
cd script
```

Then, you can just run the test :

```
python sim_torque_bellStep.py
```

This will launch the simulation making the robot executing a sinusoid movement of its left foot in the air (a "bell step") in torque control. 

The script also saves the dynamic graph in /tmp/sot_talos_tsid_bellStep.pdf.

## Other

More information on how to use the SoT and how to work on Talos can be found <a href="https://wiki.laas.fr/robots/Pyrene">in the robot wiki page</a> (you need LAAS permissions to access this).
