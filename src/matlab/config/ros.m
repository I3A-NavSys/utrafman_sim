% This file allows you to configure ROS master IP address. 
% It is useful if you work with different machines.

% Based on the username value, ROS_IP is changed.
switch  getenv("USERNAME")
    case 'Rafa'
        ROS_IP = '192.168.17.128';
    otherwise
        %Default value
        ROS_IP = '127.0.0.1';
end
