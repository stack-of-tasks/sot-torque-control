# Make Pyrene walk in torque control (quasistatic trajectories)

In the following, we demonstrate how to run the walking simulation with  <a href="https://github.com/stack-of-tasks/sot-torque-control">sot-torque-control</a>, and talos-torque-control; using the reference quasistatic trajectories computed by <a href="https://github.com/loco-3d/multicontact-api">multicontact-api</a>.

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

Then, you can just run the test, specifying which type of walk you want the robot to execute (on spot or 20cm steps):

```
Usage: python sim_walk_torque.py walk_type:=[on_spot|walk_20] {path_folder_of_the_reference_trajectories}
```
For instance, for the walk on spot simulation, just run:

```
python sim_walk_torque.py on_spot
```

This will launch the simulation making the robot walk on spot in torque control (for now only a quasistatic movement). 

The script also saves the dynamic graph in /tmp/sot_talos_tsid_walk.pdf.

For the 20cm walk just specify "walk_20" instead of "on_spot" in the command line.

If you have some reference trajectories which are not the ones of the talos-torque-control package, you can test them by specifying the absolute path of their folder:

```
python sim_walk_torque.py walk_20 path_to_folder_of_ref_trajectories
```

The trajectories must have a .dat extension and the following names:
* am.dat -> angular momentum trajectory (3D vector to 9D vector if derivatives)
* com.dat -> center of Mass trajectory (3D vector to 9D vector if derivatives)
* leftFoot.dat and rightFoot.dat -> feet trajectories (12D SE3 vector to 36D SE3 vector if derivatives)
* leftForceFoot.dat and rightForceFoot.dat -> feet forces trajectories (6D vector to 18D vector if derivatives)

## Other

More information on how to use the SoT and how to work on Talos can be found <a href="https://wiki.laas.fr/robots/Pyrene">in the robot wiki page</a> (you need LAAS permissions to access this).
