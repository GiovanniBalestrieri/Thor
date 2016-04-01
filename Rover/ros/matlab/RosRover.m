% Connect to a remote ROS instance on port 11311
% localROS , NodeHost, remote
rosinit('http://160.80.97.150:11311', 'NodeHost', '160.80.97.241')

% Check for topics
rostopic list

% Echo for /scan topic
laser = rossubscriber('/scan', rostype.sensor_msgs_LaserScan)


odom = rossubscriber('/odom')


% Receive Scan data
scandata = receive(laser,3)

% Error using robotics.ros.Subscriber/receive
% (line 291)
% The function did not receive any data and timed
% out.

rosnode list
rosnode ping /twist_to_motors

setenv('ROS_MASTER_URI','http://160.80.97.150:11311')
setenv('ROS_IP','160.80.97.241')
rosinit
rosshutdown