% Connect to a remote ROS instance on port 11311
% localROS , NodeHost, remote
rosinit('http://160.80.97.150:11311', 'NodeHost', '160.80.97.241')

% Check for topics
rostopic list

% Echo for /scan topic
laser = rossubscriber('/scan')

% Receive Scan data
scandata = receive(laser,10)

% Error using robotics.ros.Subscriber/receive
% (line 291)
% The function did not receive any data and timed
% out.