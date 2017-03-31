JID=0
if [ "$1" != "" ]; then
    echo "Going to plot data of joint $1"
    JID=$1
else
    echo "No joint number specified, going to plot data of joint 0"
fi

JID_FB=$(($JID+6))
BUFFER=30
REFRESH_RATE=10

rqt_plot traj_gen_q/data[$JID],/robotState/data[$JID_FB] \
  /traj_gen_dq/data[$JID],/estimator_jointsVelocities/data[$JID] \
  /traj_gen_ddq/data[$JID],/estimator_jointsAccelerations/data[$JID]
#~ rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE \
  #~ /traj_gen_q/data[$JID],/robotState/data[$JID_FB] \
  #~ /traj_gen_dq/data[$JID],/estimator_jointsVelocities/data[$JID] \
  #~ /traj_gen_ddq/data[$JID],/estimator_jointsAccelerations/data[$JID]

#rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='traj_gen q ref','encoder','ctrl_man q des','dq ref','dq','ddq ref','ddq','ddq ref+PD'\
#  /traj_gen_q/data[$JID],/robotState/data[$JID_FB],/ctrl_manager_qDes/data[$JID] \
#  /traj_gen_dq/data[$JID],/estimator_jointsVelocities/data[$JID] \
#  /traj_gen_ddq/data[$JID],/estimator_jointsAccelerations/data[$JID],/inv_dyn_ddqDes/data[$JID]
