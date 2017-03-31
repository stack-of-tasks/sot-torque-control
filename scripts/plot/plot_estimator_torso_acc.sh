# right foot sensor has X pointing forward, Y pointing leftward, Z pointing upward
BUFFER=30
REFRESH_RATE=10

rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='x torso acc','x imu acc','y torso acc','y imu acc','z torso acc','z imu acc' /estimator_torsoAcceleration/data[0],/accelerometer/data[0] /estimator_torsoAcceleration/data[1],/accelerometer/data[1] /estimator_torsoAcceleration/data[2],/accelerometer/data[2] &
