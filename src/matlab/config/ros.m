%This file allows you to configure ROS master IP address. Useful if you
%work different machines. Based on the username value, ROS_IP is changed.

ROS_IP = '127.0.0.1';

if getenv('USERNAME') == "jesus"
    ROS_IP = "1.0.0.131";
%     ROS_IP = "192.168.1.131";

elseif getenv("USERNAME") == "usuario"
    ROS_IP = "192.168.1.131";
end