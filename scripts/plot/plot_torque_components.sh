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

rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='tau des','tau','tau FB','tau FF'\
  /inv_dyn_tauDes/data[$JID],/estimator_jointsTorques/data[$JID],/inv_dyn_tauFB/data[$JID],/inv_dyn_tauFF/data[$JID] &
