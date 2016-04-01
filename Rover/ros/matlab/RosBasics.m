% Use rosinit to initialize ROS. By default, rosinit creates a ROS master 
% in MATLAB and starts a "global node" that is connected to the master. 
% The "global node" is automatically used by other ROS functions.
rosinit

% Use rosnode list to see all nodes in the ROS network. Note that the only 
% available node is the global node created by rosinit.
rosnode list

disp('press X');
pause();
% Use exampleHelperROSCreateSampleNetwork to populate the ROS network 
% with three additional nodes and sample publishers and subscribers.
exampleHelperROSCreateSampleNetwork

rosnode list

% Use rostopic list to see available topics in the ROS network. 
% Observe that there are three active topics: /pose, /rosout, and /scan. 
% rosout is a default logging topic that is always present in the ROS
% network. The other two topics were created as part of the sample network.

rostopic list

% Use rostopic info to get specific information about a specific topic. 
% The command below shows that /node_1 publishes (sends messages to) the 
% /pose topic, and /node_2 subscribes (receives messages from) that topic
rostopic info /pose

% Use rosnode info to get information about a specific node.
rosnode info /node_1

rosservice list
rosservice info /add

rostopic type /pose

% Use rosmsg show to view the properties of a message type. The
% geometry_msgs/Twist message type has two properties, Linear and Angular. 
% Each property is a message of type geometry_msgs/Vector3, which in turn 
% has three properties of type double.
rosmsg show geometry_msgs/Twist

rosmsg show geometry_msgs/Vector3


% se exampleHelperROSShutDownSampleNetwork to remove the sample nodes,
% publishers, and subscribers from the ROS network. 
exampleHelperROSShutDownSampleNetwork

% Use rosshutdown to shut down the ROS network in MATLAB.
rosshutdown