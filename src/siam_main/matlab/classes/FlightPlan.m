classdef FlightPlan < handle

    properties
        flightPlanId
        status = 0;
        priority = 0;

        operator;
        drone;
        %Is dtto really needed?
        dtto;

        route = ros.msggen.siam_main.Waypoint.empty;
        sent = 0;
    end
    
    methods
        function obj = FlightPlan(operator, drone, route, dtto)
            obj.operator = operator;
            obj.drone = drone;
            obj.dtto = dtto;
            
            init = ros.msggen.siam_main.Waypoint;
            init.X = drone.initLoc(1);
            init.Y = drone.initLoc(2);
            init.Z = drone.initLoc(3);
            init.T.Sec = dtto;
            init.R = 0.5;
            obj.route(1) = init;
            
            for x = 1:size(route,1)
                point = ros.msggen.siam_main.Waypoint;
                point.X = route(x,1);
                point.Y = route(x,2);
                point.Z = route(x,3);
                point.T.Sec = dtto+(x*10);
                point.R = 0.5;
                obj.route(x+1) = copy(point);
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

        function p = AbstractionLayer(fp, t)
            %If t before init Uplan, not valid pos
            if t < fp.route(1).T.Sec
                p = [0 0 0];
                return;
            end

            %If t after finishing Uplan, return finish position
            if t > fp.route(end).T.Sec
                p = [fp.route(end).X fp.route(end).Y fp.route(end).Z];
                return;
            end

            % Travel waypoint (assuming first waypoint is where drone is)
            for i = 2:size(fp.route,2)
                waypoint = fp.route(i);
                
                %until the next waypoint to t
                if t > waypoint.T.Sec
                    continue;
                end

                %Now waypoint is the next waypoint
                timeDifBetWays = waypoint.T.Sec - fp.route(i-1).T.Sec;  %Time dif between last waypoint and enroute waypoint
                timeDifBetWayT = t - fp.route(i-1).T.Sec;               %Time dif between last waypoint and t
                %Compute dif between waypoints
                vectorDif = [waypoint.X waypoint.Y waypoint.Z] - [fp.route(i-1).X fp.route(i-1).Y fp.route(i-1).Z];
                %Now vectorDIf is the RELATIVE vector between last waypoint and
                %position in t, so must be convewrted to ABSOLUTE.
                p = (vectorDif.*(timeDifBetWayT/timeDifBetWays)) + [fp.route(i-1).X fp.route(i-1).Y fp.route(i-1).Z];
                return;
            end

        end

        %Temporal
        function CheckAbstractionLayer(fp)
            figure(10);

            for i= 1:1:200
                pos(i,:) = jesus.AbstractionLayer(i);
            end

            plot3(pos(:,1),pos(:,2),pos(:,3));
        end
    end

    methods(Static)
        function route = GenerateRandomRoute(nway)
            %Airspace bounds
            bounds =   [[-4 4]
                        [4 -4]];

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

