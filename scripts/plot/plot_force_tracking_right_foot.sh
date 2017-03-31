# right foot sensor has X pointing forward, Y pointing leftward, Z pointing upward
BUFFER=30
REFRESH_RATE=10

rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='RF fx ref','RF fx','RF fy ref','RF fy','RF fz ref','RF fz' \
  /traj_gen_fRightFoot/data[0],/estimator_contactWrenchRightSole/data[0] \
  /traj_gen_fRightFoot/data[1],/estimator_contactWrenchRightSole/data[1] \
  /traj_gen_fRightFoot/data[2],/estimator_contactWrenchRightSole/data[2] &
