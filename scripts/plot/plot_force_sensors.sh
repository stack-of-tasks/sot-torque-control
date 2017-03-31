# right foot sensor has X pointing forward, Y pointing leftward, Z pointing upward
BUFFER=30
REFRESH_RATE=10

rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='fx LLEG','fx RLEG','fy LLEG','fy RLEG','fz LLEG','fz RLEG' /forceLLEG/data[0],/forceRLEG/data[0] /forceLLEG/data[1],/forceRLEG/data[1] /forceLLEG/data[2],/forceRLEG/data[2] &
