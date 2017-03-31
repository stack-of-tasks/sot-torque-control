# right foot sensor has X pointing forward, Y pointing leftward, Z pointing upward
BUFFER=30
REFRESH_RATE=10

rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='fx LARM','fx RARM','fy LARM','fy RARM','fz LARM','fz RARM' /forceLARM/data[0],/forceRARM/data[0] /forceLARM/data[1],/forceRARM/data[1] /forceLARM/data[2],/forceRARM/data[2] &
