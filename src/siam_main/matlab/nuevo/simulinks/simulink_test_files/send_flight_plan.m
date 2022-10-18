topico = "/planesvuelo";
r = 2;

try
    rosinit("192.168.1.131",11311);
catch
    disp("ROS ya inicializado");
end

pub = rospublisher(topico, "siam_main/FlightPlan");
msg = rosmessage('siam_main/FlightPlan');
point = rosmessage("geometry_msgs/Point");

msg.FlightPlanId = 100;
msg.Status = 0;
msg.Priority = 7;
msg.OperatorId = 10;
msg.DroneId = 1000;
msg.Dtto = 5430;

if r==1
    route =     [[0 4 1]
                [9 4 1]
                [9 -4 1]
                [0 -4 1]];
else

    route =     [[0 -4 1]
                [9 -4 1]
                [9 4 1]
                [0 4 1]];
end

point.X = route(1,1);
point.Y = route(1,2);
point.Z = route(1,3);

msg.Orig = copy(point);

point.X = route(end,1);
point.Y = route(end,2);
point.Z = route(end,3);

msg.Dest = copy(point);

for i = 1:1:length(route)
    point.X = route(i,1);
    point.Y = route(i,2);
    point.Z = route(i,3);

    msg.Route(i) = copy(point);
end

send(pub,msg);
