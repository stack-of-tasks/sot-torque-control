#connect to mocap
from dynamic_graph.ros import RosExport
rosIn = RosExport('rosExportMocap')
rosIn.add('matrixHomoStamped', "chest", "/evart/chest9M/chest9M")
robot.device.after.addDownsampledSignal('rosExportMocap.chest',1)
rosIn.add('matrixHomoStamped', "left_foot", "/evart/hrp2_14_LeftFoot/hrp2_14_LeftFoot")
robot.device.after.addDownsampledSignal('rosExportMocap.left_foot',1)
rosIn.add('matrixHomoStamped', "left_hand", "/evart/hand_flag4m/hand_flag4m")
robot.device.after.addDownsampledSignal('rosExportMocap.left_hand',1)

#republish mocap
ros.add('matrix', 'chest_ros',      'chest');
plug(rosIn.chest,    ros.chest_ros);



tracer=create_tracer(robot.device)
addTrace(tracer,rosIn,'chest')
addTrace(tracer,rosIn,'left_foot')
addTrace(tracer,rosIn,'left_hand')


#MOCAP:**************************************
ssh tflayols@hakusan-vm
export ROS_MASTER_URI=http://hrp2014c:11311
roslaunch evart_bridge start.launch&
rosservice call /evart/list_segments
rosservice call /evart/track_segments chest9M chest9M
rosservice call /evart/track_segments hand_flag4m hand_flag4m
rosservice call /evart/track_segments hrp2_14_LeftFoot hrp2_14_LeftFoot
