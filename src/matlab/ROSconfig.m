%This file allows you to configure ROS master IP address. Useful if you
%work different machines. Based on the username value, ROS_IP is changed.

if getenv('USERNAME') == "jesus"
    ROS_IP = "1.0.0.131";
%     ROS_IP = "192.168.1.131";

elseif getenv("USERNAME") == "usuario"
    ROS_IP = "192.168.1.131";

elseif getenv('USERNAME') == "siamsim"
    ROS_IP = 'localhost';

elseif getenv("USERNAME") == 'galadriel-ubunut'
    ROS_IP = 'localhost';
end