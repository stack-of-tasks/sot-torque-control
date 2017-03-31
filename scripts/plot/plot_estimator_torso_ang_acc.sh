BUFFER=30
REFRESH_RATE=10

rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='x torso ang acc','y torso ang acc','z torso ang acc' /estimator_torsoAcceleration/data[3] /estimator_torsoAcceleration/data[4] /estimator_torsoAcceleration/data[5] &
