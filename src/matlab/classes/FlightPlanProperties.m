%This class defines the FlightPlan object, which is used to store the flight plan information and to parse it to ROS messages.

classdef FlightPlanProperties < handle
    properties
        flightplan_id uint32        %Unique ID for the flight plan
        status int8 = 0             %Status flag (to mark if the flight plan is waiting, in progress or finished)
        priority uint8 = 0          %NOT USED actually

        operator Operator           %Operator object reference
        uav UAVProperties           %Drone object reference
        dtto double                 %Desired time to take off (where the plan starts)

        route = ros.msggen.utrafman_main.Waypoint.empty;        %Array of ROS Waypoint messages defining the route of the flight plan
        sent uint8 = 0                                          %Sent status flag (to mark if the flight plan has been sent to the drone)
    end
    
    methods
        %Class constructor
        function obj = FlightPlanProperties(operator, drone, route, dtto)
            vel = 2;

            %Assigning references
            obj.operator = operator;
            obj.uav = drone;
            obj.dtto = dtto;
            accumt = 0;
            
            %Generating first waypoint (where the drone is before take off)
            init = ros.msggen.utrafman_main.Waypoint;
            init.X = drone.init_loc(1);
            init.Y = drone.init_loc(2);
            init.Z = drone.init_loc(3);
            init.T.Sec = dtto;
            init.R = 0.5;
            obj.route(1) = init;
            
            %For each waypoint in route, create a ROS Waypoint message and add it to the route array
            for x = 1:size(route,1)
                point = ros.msggen.utrafman_main.Waypoint;
                point.X = route(x,1);
                point.Y = route(x,2);
                point.Z = route(x,3);

                %If the route is defined with time, use it, otherwise use the default time interval of 10 seconds
                if (length(route(x,:)) == 4)
                    point.T.Sec = route(x,4);
                else
                    if (x == 1)
                        dist = norm(drone.init_loc - route(x,:));
                        time_diff = ceil(dist/vel)+1;
                        accumt = accumt + time_diff;
                        point.T.Sec = dtto + accumt;
                    else
                        dist = norm(route(x-1,:) - route(x,:));
                        time_diff = ceil(dist/vel);
                        accumt = accumt + time_diff;
                        point.T.Sec = dtto + accumt;
                    end
                end
                point.R = 0.5;
                %Add waypoint to route
                obj.route(x+1) = copy(point);
            end

            %Last waypoint
            last = ros.msggen.utrafman_main.Waypoint;
            last.X = route(end,1);
            last.Y = route(end,2);
            last.Z = 1;
            last.R = 0.5;
            dist = norm(route(end,:) - [last.X last.Y last.Z]);
            time_diff = ceil(dist/vel);
            accumt = accumt + time_diff;
            last.T.Sec = dtto + accumt;
            obj.route(end+1) = last;

        end


        %Function to parse FlightPlan object to ROS object
        function msg = parseToROSMessage(obj)
            %Generating ROS messages
            msg = rosmessage('utrafman_main/Uplan');
            point = rosmessage("utrafman_main/Waypoint");
            time = rosmessage("std_msgs/Time");
            %Assigning values
            if ~isempty(obj.flightplan_id)
                msg.FlightPlanId = obj.flightplan_id;
            end
            msg.Status = obj.status;
            msg.Priority = obj.priority;
            msg.OperatorId = obj.operator.operator_id;
            msg.DroneId = obj.uav.drone_id;
            msg.Dtto = obj.dtto;
            msg.Route = obj.route;
        end 

        %Implementation of the AbstractionLayer for flight plans
        %This function returns the position of the drone at any time t interpolating the route waypoints.
        %Allow abstraction of the flight plan definition (waypoints) and the real flight plan execution (position of the drone at any time).
        function p = abstractionLayer(fp, t)
            %If t is before init Uplan, return not valid pos
            if t < fp.route(1).T.Sec
                p = [0 0 0];
                return;
            end

            %If t after finishing Uplan, return finish position
            if t > fp.route(end).T.Sec
                p = [fp.route(end).X fp.route(end).Y fp.route(end).Z];
                return;
            end

            % Travel waypoint (assuming first waypoint is where drone is before take off)
            for i = 2:size(fp.route,2)
                waypoint = fp.route(i);
                
                %Until the next waypoint to t
                if t > waypoint.T.Sec
                    continue;
                end

                %Now waypoint is the next waypoint
                timeDifBetWays = waypoint.T.Sec - fp.route(i-1).T.Sec;  %Time dif between last waypoint and enroute waypoint
                timeDifBetWayT = t - fp.route(i-1).T.Sec;               %Time dif between last waypoint and t
                %Compute dif between waypoints
                vectorDif = [waypoint.X waypoint.Y waypoint.Z] - [fp.route(i-1).X fp.route(i-1).Y fp.route(i-1).Z];
                %Now vectorDif is the RELATIVE vector between last waypoint and
                %position in t, so must be converted to ABSOLUTE.
                p = (vectorDif.*(timeDifBetWayT/timeDifBetWays)) + [fp.route(i-1).X fp.route(i-1).Y fp.route(i-1).Z];
                return;
            end
        end
    end

    methods(Static)
        %No longer in use.
        %This function is used to generate random flight plans inside bounds and with a given number of waypoints.
        %Used for testing purposes.
        function route = generateRandomRoute(nway)
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

        function p = abstractionLayerUplan(uplan, t)
            %If t is before init Uplan, return not valid pos
            if t < uplan.Route(1).T.Sec
                p = [0 0 0];
                return;
            end

            %If t after finishing Uplan, return finish position
            if t > uplan.Route(end).T.Sec
                p = [uplan.Route(end).X uplan.Route(end).Y uplan.Route(end).Z];
                return;
            end

            % Travel waypoint (assuming first waypoint is where drone is before take off)
            for i = 2:size(uplan.Route,1)
                waypoint = uplan.Route(i);
                
                %Until the next waypoint to t
                if t > waypoint.T.Sec
                    continue;
                end

                %Now waypoint is the next waypoint
                timeDifBetWays = waypoint.T.Sec - uplan.Route(i-1).T.Sec;  %Time dif between last waypoint and enroute waypoint
                timeDifBetWayT = t - uplan.Route(i-1).T.Sec;               %Time dif between last waypoint and t
                %Compute dif between waypoints
                vectorDif = [waypoint.X waypoint.Y waypoint.Z] - [uplan.Route(i-1).X uplan.Route(i-1).Y uplan.Route(i-1).Z];
                %Now vectorDif is the RELATIVE vector between last waypoint and
                %position in t, so must be converted to ABSOLUTE.
                p = (vectorDif.*(timeDifBetWayT/timeDifBetWays)) + [uplan.Route(i-1).X uplan.Route(i-1).Y uplan.Route(i-1).Z];
                return;
            end
        end
    end
end

