bgp = gcp('nocreate');
parfeval(bgp,@mensajero,0)

function mensajero()
    node = ros.Node("mensajero","192.168.1.131",11311);
    pub = ros.Publisher(node,"/mensajero","std_msgs/String");
    msg = rosmessage(pub);
    msg.Data = "Jesus";
    
    for i=1:1:3000
        pub.send(msg)
        pause(5);
    end
end

