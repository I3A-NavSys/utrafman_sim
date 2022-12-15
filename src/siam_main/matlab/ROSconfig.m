if getenv('USERNAME') == "jesus"
    ROS_IP = "1.0.0.131";
    ROS_IP = "192.168.1.131";
elseif getenv("USERNAME") == "usuario"
    ROS_IP = "192.168.1.131";
end