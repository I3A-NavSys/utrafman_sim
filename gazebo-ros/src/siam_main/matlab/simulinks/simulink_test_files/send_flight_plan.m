drones = 1;
fp_per_drone = 1;

bounds = [[-1 -5]
          [9 5]];

pub = ros.Publisher.empty;
msg = ros.msggen.siam_main.FlightPlan.empty;

try
    rosinit("192.168.1.131",11311);
catch
    disp("ROS ya inicializado");
end

for e = 1:fp_per_drone %Planes de vuelos por drone
    for i=1:drones %Drones
        topico = "/test/"+i+"/flightPlans/request";
        topico2 = "/test/"+i+"/flightPlans/";
        
        pub(i) = rospublisher(topico, "siam_main/FlightPlan");
        pub3 = rospublisher(topico2, "siam_main/FlightPlan");
        msg(i) = rosmessage('siam_main/FlightPlan');
        point = rosmessage("geometry_msgs/Point");
        
        msg(i).FlightPlanId = 100;
        msg(i).Status = 0;
        msg(i).Priority = 7;
        msg(i).OperatorId = 10;
        msg(i).DroneId = 1000;
        msg(i).Dtto = 1;
        
        route = [];
        for j = 1:randi(5)+3
            x = bounds(2,1) * rand(1) + bounds(1,1);
            y = bounds(2,2) * rand(1) + bounds(1,2);
            z = 4 * rand(1) + 1;
    
            route(j,:) = [x y z];
        end
        
        point.X = route(1,1);
        point.Y = route(1,2);
        point.Z = route(1,3);
        
        msg(i).Orig = copy(point);
        
        point.X = route(end,1);
        point.Y = route(end,2);  
        point.Z = route(end,3);
        
        msg(i).Dest = copy(point);
        
        for x = 1:1:length(route)
            point.X = route(x,1);
            point.Y = route(x,2);
            point.Z = route(x,3);
        
            msg(i).Route(x) = copy(point);
        end
        send(pub(i),msg(i));
    end
end

