%bgp = parpool('Processes');

for i=1:1:1
     bgt(i) = parfeval(bgp,@initros,0,"node2_"+i);
end

%initros("node3");

function printClock(sub, msg, pub)
    %disp(msg.Clock_.Nsec);
    msg = rosmessage("std_msgs/String");
    msg.Data = 'Jesus Jover';
    send(pub,msg);
end

function initros(name)
    rosshutdown;
    rosinit("192.168.1.131",11311,"NodeName",name);
    pubs = rospublisher("/receptor","std_msgs/String");
    subs = rossubscriber("/mensajero","std_msgs/String",{@printClock,pubs});

    %sub_node = ros.Node(name,"192.168.1.131",11311);
    %subcriber = ros.Subscriber('/clock','rosgraph_msgs/Clock',@printClock);
%     t = timer("Period",1000,"TimerFcn",@(~,~)disp(''));
%     start(t);
    pause(Inf);
end