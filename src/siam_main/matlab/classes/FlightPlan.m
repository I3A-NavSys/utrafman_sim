classdef FlightPlan < handle

    properties
        flightPlanId
        status = 0;
        priority = 0;

        operator;
        drone;

        %orig
        %dest
        dtto;

        route = ros.msggen.siam_main.Waypoint.empty;
        sent = 0;
    end
    
    methods
        function obj = FlightPlan(operator, drone, route, dtto)
            obj.operator = operator;
            obj.drone = drone;
            obj.dtto = dtto;
            
            for x = 1:1:length(route)
                point = ros.msggen.siam_main.Waypoint;
                point.X = route(x,1);
                point.Y = route(x,2);
                point.Z = route(x,3);
                point.T.Sec = obj.dtto+(15*x);
                point.R = 0.5;
                obj.route(x) = copy(point);
            end

        end

        function msg = parseToROSMessage(obj)
            %Generating ros messages
            msg = rosmessage('siam_main/Uplan');
            point = rosmessage("siam_main/Waypoint");
            time = rosmessage("std_msgs/Time");

            msg.FlightPlanId = obj.flightPlanId;
            msg.Status = obj.status;
            msg.Priority = obj.priority;
            msg.OperatorId = obj.operator.operatorId;
            msg.DroneId = obj.drone.droneId;
            msg.Dtto = obj.dtto;

            msg.Route = obj.route;

        end 
    end

    methods(Static)
        function route = GenerateRandomRoute(nway)
            %Airspace bounds
            bounds =   [[-5 5]
                        [5 -5]];

            %Waypoints
            route = zeros(nway,3);
            for j = 1:nway
                x = (bounds(2,1)-bounds(1,1)) * rand(1) + bounds(1,1);
                y = (bounds(2,2)-bounds(1,2)) * rand(1) + bounds(1,2);
                z = 4 * rand(1) + 1;
        
                route(j,:) = [x y z];
            end
        end
    end
end

