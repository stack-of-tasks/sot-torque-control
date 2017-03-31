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

rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='position','velocity','acceleration','torque' \
  /robotState/data[$JID_FB] \
  /estimator_jointsVelocities/data[$JID] /estimator_jointsAccelerations/data[$JID] /estimator_jointsTorques/data[$JID] &
