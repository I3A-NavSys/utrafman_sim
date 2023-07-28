classdef FlightPlan < handle
    %FLIGHTPLAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id int8 = 0;
        waypoints Waypoint = Waypoint.empty;
        init_time double;
        priority int8 = 0; 
    end
    
    methods
        function obj = FlightPlan(waypoints, init_time, priority)
            %FLIGHTPLAN Construct for FlightPlan class

            %Check if the waypoints list is empty ([])
            if ~isempty(waypoints)
                %Add waypoints to the flight plan
                for i = 1:length(waypoints)
                    obj.addWaypoint(waypoints(i));
                end
            end

            obj.init_time = init_time;
            obj.priority = priority;
        end

        function obj = changeInitTime(obj,init_time)
            %CHANGEINITTIME This method allow to change the init time of a flight plan

            obj.init_time = init_time;
        end

        function obj = changePriority(obj,priority)
            %CHANGEPRIORITY This method allow to change the priority of a flight plan
            %   Detailed explanation goes here

            obj.priority = priority;
        end
        
        function obj = addWaypoint(obj,waypoint)
            %ADDWAYPOINT This method allow to add a waypoint to a flight plan
            %   Inserting waypoints out of order are allowed using this method
            
            %Check if the waypoint is a Waipoint object
            if ~isa(waypoint,'Waypoint')
                error('The waypoint must be a Waipoint object');
            end

            %Check if the waypoint is already in the flight plan
            if ~isempty(obj.waypoints)
                for i = 1:length(obj.waypoints)
                    if obj.waypoints(i).isequal(waypoint)
                        return %The waypoint is already in the flight plan
                    end
                end
            end

            %Add the waypoint to the flight plan (depending on the time)
            if isempty(obj.waypoints)
                obj.waypoints = waypoint;
            else
                for i = 1:length(obj.waypoints)
                    if obj.waypoints(i).t > waypoint.t
                        obj.waypoints = [obj.waypoints(1:i-1), waypoint, obj.waypoints(i:end)];
                        return %The waypoint is added to the flight plan (out of order)
                    end
                end
                obj.waypoints = [obj.waypoints, waypoint]; %The waypoint is added to the end of the flight plan
            end
        end

        function obj = removeWaypoint(obj,waypoint)
            %REMOVEWAYPOINT This method allow to remove a waypoint from a flight plan
            %   Removing waypoints out of order are allowed using this method
            
            %Check if the waypoint is a Waipoint object
            if ~isa(waypoint,'Waypoint')
                error('The waypoint must be a Waipoint object');
            end

            %Check if the waypoint is already in the flight plan
            if ~isempty(obj.waypoints)
                for i = 1:length(obj.waypoints)
                    if obj.waypoints(i).isequal(waypoint)
                        obj.waypoints(i) = []; %The waypoint is removed from the flight plan
                        return
                    end
                end
            end
        end

        function obj = updateWaypoint(obj,waypoint)
            %UPDATEWAYPOINT This method allow to update a waypoint from a flight plan
            %   Updating waypoints out of order are allowed using this method
            
            %Check if the waypoint is a Waipoint object
            if ~isa(waypoint,'Waypoint')
                error('The waypoint must be a Waipoint object');
            end

            %Check if the waypoint is already in the flight plan
            if ~isempty(obj.waypoints)
                for i = 1:length(obj.waypoints)
                    if obj.waypoints(i).isequal(waypoint)
                        obj.waypoints(i) = waypoint; %The waypoint is updated from the flight plan
                        return
                    end
                end
            end
        end

        function obj = clearWaypoints(obj)
            %CLEARWAYPOINTS This method allow to clear all the waypoints from a flight plan
            
            obj.waypoints = Waipoint.empty;
        end

        function plt = plotRoute(obj, color)
            %PLOTFIGURE This method add the flight plan to the current figure
            
            %If color is empty, set the default color to blue
            if isempty(color)
                color = 'b';
            end
            
            %Display waypoints
            plt = plot3([obj.waypoints(:).x], [obj.waypoints(:).y], [obj.waypoints(:).z], 'o', 'MarkerSize', 5, 'MarkerFaceColor', color);
            
            %Display routes between waypoints
            for i = 1:length(obj.waypoints)-1
                plot3([obj.waypoints(i).x, obj.waypoints(i+1).x], [obj.waypoints(i).y, obj.waypoints(i+1).y], [obj.waypoints(i).z, obj.waypoints(i+1).z], 'Color', color);
            end
        end

        function obj = routeFigure(obj)
            %DISPLAYFIGURE This method allow to display the flight plan
            
            %Check if the flight plan is empty
            if isempty(obj.waypoints)
                disp('The flight plan is empty');
                return
            end

            %Find if the figure is already open
            fig_name = "Route for flight plan ID " + obj.id;
            fig = findobj("Name", fig_name);
            if isempty(fig)
                %Display a figure with the flight plan
                fig = figure("Name", fig_name);
            else
                %Select the figure
                figure(fig)
                clf(fig)
            end

            %Figure settings
            hold on;
            grid on;
            axis equal;
            xlabel("x [m]");
            ylabel("y [m]");
            zlabel("z [m]");
            title("Route for flight plan ID " + obj.id);
            obj.plotRoute('b'); 
        end

        function obj = velocityFigure(obj)
            %DISPLAYVELOCITY This method allow to display the velocity of the flight plan
            
            %Check if the flight plan is empty
            if isempty(obj.waypoints)
                disp('The flight plan is empty');
                return
            end

            %Find if the figure is already open
            fig_name = "Velocity for flight plan ID " + obj.id;
            fig = findobj('Type', 'Figure',"Name", fig_name);
            if isempty(fig)
                %Display a figure with the flight plan
                fig = figure("Name", fig_name);
                fig.Name = "Velocity for flight plan ID " + obj.id;
            else
                %Select the figure
                figure(fig)
                clf(fig)
            end

            %Figure settings
            hold on;
            grid on;
            xlabel("Time [s]");
            ylabel("Velocity [m/s]");
            title("Velocity for flight plan ID " + obj.id);

            %Compute the velocity between waypoints
            velocity = [];
            for i = 1:length(obj.waypoints)-1
                velocity = [velocity, obj.waypoints(i).velocityWithWaypoint(obj.waypoints(i+1))];
            end

            %Display velocity
            plot([obj.waypoints(2:end).t], velocity);
        end

        function obj = reverseWaypoints(obj)
            %REVERSE This method allow to reverse the flight plan
            %It reverse the order of the waypoints keeping the same time of the previous waypoints, but not take into account if new section velocities between waypoints are fulfillable
            
            %Check if the flight plan is empty
            if isempty(obj.waypoints)
                disp('The flight plan is empty');
                return
            end

            %Reverse the flight plan
            reversed = obj.waypoints(end:-1:1);

            for i = 1:length(reversed)
                %Change the time of the waypoints
                reversed(i).t = obj.waypoints(end-(i-1)).t;
            end

            obj.waypoints = reversed;
        end

        function obj = normalizeVelocity(obj)
            %NORMALIZEVELOCITY This method allow to normalize the velocity of the flight plan
            %It normalize the velocity of the flight plan, doing that the velocity between waypoints to be the same for all the flight plan
            
            %Check if the flight plan is empty
            if isempty(obj.waypoints)
                disp('The flight plan is empty');
                return
            end

            %Init and finish time of the flight plan
            init_time = obj.waypoints(1).t;
            finish_time = obj.waypoints(end).t;

            %Compute the total distance of the flight plan
            total_distance = 0;
            for i = 1:length(obj.waypoints)-1
                total_distance = total_distance + obj.waypoints(i).distanceWithWaypoint(obj.waypoints(i+1));
            end

            %Compute mean velocity
            mean_velocity = total_distance/(finish_time-init_time);

            %Normalize the velocity of the flight plan
            for i = 2:length(obj.waypoints)
                %Compute the distance between waypoints
                distance = obj.waypoints(i).distanceWithWaypoint(obj.waypoints(i-1));

                %Compute the new time between waypoints
                new_time = obj.waypoints(i-1).t + distance/mean_velocity;

                %Change the time of the waypoints
                obj.waypoints(i).t = new_time;
            end
        end

        function eq = isequal(obj, flightplan)
            %ISEQUAL This method allow to compare two flight plans
            %   Two flight plans are equal if they have the same waypoints in the same order
            
            %Check if the flight plan is a FlightPlan object
            if ~isa(flightplan,'FlightPlan')
                error('The flight plan must be a FlightPlan object');
            end

            %Check if the flight plan have the same number of waypoints
            if length(obj.waypoints) ~= length(flightplan.waypoints)
                eq = false;
                return
            end

            %Check if the flight plan have the same waypoints
            for i = 1:length(obj.waypoints)
                if ~obj.waypoints(i).isequal(flightplan.waypoints(i))
                    eq = false;
                    return
                end
            end

            %Check if the flight plan have the same init_time
            if obj.init_time ~= flightplan.init_time
                eq = false;
                return
            end

            %Check if the flight plan have the same priority
            if obj.priority ~= flightplan.priority
                eq = false;
                return
            end

            eq = true;
        end

        function p = abstractionLayer(fp, t)
            %If t is before init flight plan, return not valid pos
            if t < fp.init_time
                p = [0 0 0];
                return;
            end

            %If t after finishing Uplan, return finish position
            if t > fp.waypoints(end).t
                p = [fp.waypoints(end).x fp.waypoints(end).y fp.waypoints(end).z];
                return;
            end

            % Travel waypoint (assuming first waypoint is where drone is before take off)
            for i = 2:size(fp.waypoints)
                waypoint = fp.waypoints(i);
                
                %Until the next waypoint to t
                if t > waypoint.t
                    continue;
                end

                %Now waypoint is the next waypoint
                timeDifBetWays = waypoint.t - fp.waypoints(i-1).t;      %Time dif between last waypoint and enroute waypoint
                timeDifBetWayT = t - fp.waypoints(i-1).t;               %Time dif between last waypoint and t

                %Compute dif between waypoints
                vectorDif = [waypoint.x waypoint.y waypoint.z] - [fp.waypoints(i-1).x fp.waypoints(i-1).y fp.waypoints(i-1).z];
                %Now vectorDif is the RELATIVE vector between last waypoint and
                %position in t, so must be converted to ABSOLUTE.
                p = (vectorDif.*(timeDifBetWayT/timeDifBetWays)) + [fp.waypoints(i-1).x fp.waypoints(i-1).y fp.waypoints(i-1).z];
                return;
            end
        end
    end
end

