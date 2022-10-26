classdef FlightPlan < handle

    properties
        flightPlanId
        status = 0;
        priority = 0;

        operator
        drone

        orig
        dest
        dtto

        route

    end
    
    methods
        function obj = FlightPlan(operator, drone, orig, dest, route, dtto)
            obj.operator = operator;
            obj.drone = drone;
            obj.orig = orig;
            obj.dest = dest;
            obj.route = route;
            obj.dtto = dtto;
        end

        function msg = parseToROSMessage(obj)

            msg = rosmessage('siam_main/FlightPlan');
            point = rosmessage("geometry_msgs/Point");

            msg.FlightPlanId = obj.flightPlanId;
            msg.Status = obj.status;
            msg.Priority = obj.priority;
            msg.OperatorId = obj.operator.operatorId;
            msg.DroneId = obj.drone.droneId;
            msg.Dtto = obj.dtto;

            point.X = obj.route(1,1);
            point.Y = obj.route(1,2);
            point.Z = obj.route(1,3);
            
            msg.Orig = copy(point);
            
            point.X = obj.route(end,1);
            point.Y = obj.route(end,2);  
            point.Z = obj.route(end,3);
            
            msg.Dest = copy(point);
            
            for x = 1:1:length(obj.route)
                point.X = obj.route(x,1);
                point.Y = obj.route(x,2);
                point.Z = obj.route(x,3);
            
                msg.Route(x) = copy(point);
            end

        end 
    end

    methods(Static)
        function route = GenerateRandomRoute(nway)
            bounds = [  [-1 -5]
                        [9 5]];
            route = zeros(nway,3);
            for j = 1:nway
                x = bounds(2,1) * rand(1) + bounds(1,1);
                y = bounds(2,2) * rand(1) + bounds(1,2);
                z = 4 * rand(1) + 1;
        
                route(j,:) = [x y z];
            end
        end
    end
end

