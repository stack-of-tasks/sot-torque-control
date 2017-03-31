# right foot sensor has X pointing forward, Y pointing leftward, Z pointing upward
BUFFER=30
REFRESH_RATE=10

rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='prediction','fx RLEG','prediction','fy LLEG','prediction','fz RLEG' \
  /estimator_ftSensRightFootPrediction/data[0],/forceRLEG/data[0] \
  /estimator_ftSensRightFootPrediction/data[1],/forceRLEG/data[1] \
  /estimator_ftSensRightFootPrediction/data[2],/forceRLEG/data[2] &
