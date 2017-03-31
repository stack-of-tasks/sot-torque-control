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

export ROS_MASTER_URI=http://hrp2014c:11311/
rqt_plot /inv_dyn_tauDes/data[$JID], /estimator_jointsTorques/data[$JID]

#rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='tau des','tau','qRef-q','delta_q FB','delta_q FF','delta_q FB','delta_q friction','delta q'\
 # /inv_dyn_tauDes/data[$JID],/estimator_jointsTorques/data[$JID] \
  #/inv_dyn_qError/data[$JID],/torque_ctrl_deltaQ_fb/data[$JID] \
  #/torque_ctrl_deltaQ_ff/data[$JID],/torque_ctrl_deltaQ_fb/data[$JID],/torque_ctrl_deltaQ_friction/data[$JID],/torque_ctrl_deltaQ/data[$JID] &
