# right foot sensor has X pointing forward, Y pointing leftward, Z pointing upward
BUFFER=30
REFRESH_RATE=10

rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='x torso ang vel','x imu ang vel','y torso ang vel','y imu ang vel','z torso ang vel','z imu ang vel' /estimator_torsoAngularVelocity/data[0] /gyrometer/data[0] /estimator_torsoAngularVelocity/data[1] /gyrometer/data[1] /estimator_torsoAngularVelocity/data[2] /gyrometer/data[2] &
