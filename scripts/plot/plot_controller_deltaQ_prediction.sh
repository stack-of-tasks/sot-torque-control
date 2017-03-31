JID=0
if [ "$1" != "" ]; then
    echo "Going to plot data of joint $1"
    JID=$1
else
    echo "No joint number specified, going to plot data of joint 0"
fi

BUFFER=30
REFRESH_RATE=10

rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='delta q','predicted delta q','predicted delta q tau','predicted delta q friction','tau pred','tau' \
  /torque_ctrl_deltaQ/data[$JID],/torque_ctrl_predictedDeltaQ/data[$JID] \
  /torque_ctrl_predictedDeltaQ_tau/data[$JID],/torque_ctrl_deltaQ_friction/data[$JID] \
  /torque_ctrl_predictedJointsTorques/data[$JID],/estimator_jointsTorques/data[$JID]&
