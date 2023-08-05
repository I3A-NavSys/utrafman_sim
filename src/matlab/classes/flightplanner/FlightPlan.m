classdef FlightPlan < handle
    %FLIGHTPLAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id          int32    = 0;
        waypoints   Waypoint = Waypoint.empty;     
        priority    int8     = 0; 
    end
    
    methods

        function obj = FlightPlan(id, waypoints, priority)
            %FLIGHTPLAN Construct for FlightPlan class

            obj.id = id;
            obj.priority = priority;

            %Check if the waypoints list is empty ([])
            if ~isempty(waypoints)
                %Add waypoints to the flight plan
                for i = 1:length(waypoints)
                    obj.setWaypoint(waypoints(i));
                end
            end
        end

        
        function setWaypoint(obj,waypoint)
            %SETWAYPOINT This method allow to add/modify a waypoint into a flight plan
            %   Inserting waypoints out of order are allowed using this method
            
            %Check if the waypoint is a Waypoint object
            if ~isa(waypoint,'Waypoint')
                error('The waypoint must be a Waypoint object');
            end

            l = length(obj.waypoints);
            for i = 1:l
                if obj.waypoints(i).t == waypoint.t
                    obj.waypoints(i) = waypoint;
                    return
                end
                if obj.waypoints(i).t > waypoint.t
                    obj.waypoints = [obj.waypoints(1:i-1)  waypoint  obj.waypoints(i:l)];
                    return
                end
            end
            obj.waypoints(l+1) = waypoint;
        end


        function removeWaypointAtTime(obj,t)
            %REMOVEWAYPOINT This method allow to remove a waypoint from a flight plan
            %   Removing waypoints out of order are allowed using this method
            
            l = length(obj.waypoints);
            for i = 1:l
                if obj.waypoints(i).t == t
                    if l == 1
                        obj.waypoints = Waypoint.empty;
                    else
                        obj.waypoints = [obj.waypoints(1:i-1) obj.waypoints(i+1:end)];
                    end
                    return
                end
            end
        end


        function t = init_time(obj)
        %time of the first waypoint
            if isempty(obj.waypoints)
                t = [];
            else
                t = obj.waypoints(1).t;
            end
        end


        function t = finish_time(obj)
        %time of the last waypoint
            if isempty(obj.waypoints)
                t = [];
            else
                t = obj.waypoints(end).t;
            end        
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


        % function eq = isequal(obj, flightplan)
        %     %ISEQUAL This method allow to compare two flight plans
        %     %   Two flight plans are equal if they have the same waypoints in the same order
        % 
        %     %Check if the flight plan is a FlightPlan object
        %     if ~isa(flightplan,'FlightPlan')
        %         error('The flight plan must be a FlightPlan object');
        %     end
        % 
        %     %Check if the flight plan have the same number of waypoints
        %     if length(obj.waypoints) ~= length(flightplan.waypoints)
        %         eq = false;
        %         return
        %     end
        % 
        %     %Check if the flight plan have the same waypoints
        %     for i = 1:length(obj.waypoints)
        %         if ~obj.waypoints(i).isequal(flightplan.waypoints(i))
        %             eq = false;
        %             return
        %         end
        %     end
        % 
        %     %Check if the flight plan have the same init_time
        %     if obj.init_time ~= flightplan.init_time
        %         eq = false;
        %         return
        %     end
        % 
        %     %Check if the flight plan have the same priority
        %     if obj.priority ~= flightplan.priority
        %         eq = false;
        %         return
        %     end
        % 
        %     eq = true;
        % end


        function p = positionAtTime(obj, t)
            % Check if t is out of flight plan schedule, returning not valid pos
            if t < obj.init_time  ||  t > obj.finish_time
                p = [NaN NaN NaN];
                return;
            end

            % search for the current waypoints
            for i = 2:length(obj.waypoints)
                if t < obj.waypoints(i).t
                    break;
                end
            end

            wp1 = obj.waypoints(i-1);
            wp2 = obj.waypoints(i);

            wp3 = wp1.interpolation4D(wp2,t);
            p = wp3.position;
     
        end


        % function p = positionAtTime(fp, t)
        %     p = [Inf Inf Inf];
        % 
        %     %If t is before init flight plan, return not valid pos
        %     if t < fp.init_time
        %         return;
        %     end
        % 
        %     %If t after finishing Uplan, return finish position
        %     if t > fp.finish_time
        %         return;
        %     end
        % 
        %     % Travel waypoint (assuming first waypoint is where drone is before take off)
        %     for i = 2:size(fp.waypoints,2)
        %         waypoint = fp.waypoints(i);
        % 
        %         %Until the next waypoint to t
        %         if t > waypoint.t
        %             continue;
        %         end
        % 
        %         %Now waypoint is the next waypoint
        %         timeDifBetWays = waypoint.t - fp.waypoints(i-1).t;      %Time dif between last waypoint and enroute waypoint
        %         timeDifBetWays = fp.waypoints(i-1).timeTo(waypoint);    %Time dif between last waypoint and enroute waypoint
        %         timeDifBetWayT = t - fp.waypoints(i-1).t;               %Time dif between last waypoint and t
        % 
        %         %Compute dif between waypoints
        %         % vectorDif = [waypoint.x waypoint.y waypoint.z] - [fp.waypoints(i-1).x fp.waypoints(i-1).y fp.waypoints(i-1).z];
        %         vectorDif = waypoint.position - fp.waypoints(i-1).position;
        % 
        %         %Now vectorDif is the RELATIVE vector between last waypoint and
        %         %position in t, so must be converted to ABSOLUTE.
        %         p = (vectorDif.*(timeDifBetWayT/timeDifBetWays)) + [fp.waypoints(i-1).x fp.waypoints(i-1).y fp.waypoints(i-1).z];
        %         return;
        %     end
        % end


        function tr = trace(obj, time_step)
            %TRACE This method expands the flight plan behavior over time
            
            instants = obj.init_time : time_step : obj.finish_time;
            tr = zeros(length(instants),5);
            tr(:,1) = instants;
           
            %Get position in time instants
            for i = 1:length(instants)
                p = obj.positionAtTime(tr(i,1));
                tr(i,2:4) = p;
            end

            %Get velocity in time instants
            for i = 1:length(instants)-1
                p = obj.positionAtTime(tr(i,1));
                tr(i,5) = norm(tr(i+1,2:4) - tr(i,2:4)) / time_step;
            end
        end


        function dist = distanceTo(fp1, fp2, time_step)
            %DISTANCETO This method obstains relative distance between two flight plans over time
            
            init_time   = min(fp1.init_time,  fp2.init_time);
            finish_time = max(fp1.finish_time,fp2.finish_time);
            
            instants = init_time : time_step : finish_time;
            dist = zeros(length(instants),2);
            dist(:,1) = instants;
           
            %Get position in time instants
            for i = 1:length(instants)
                t = dist(i,1);
                p1 = fp1.positionAtTime(t);
                p2 = fp2.positionAtTime(t);
                dist(i,2) = norm(p2-p1);
            end
        end

         
        % function plt = plotRoute(obj, color)
        %     %PLOTFIGURE This method add the flight plan to the current figure
        %     %ERROR A CORREGIR. REVIENTA CUANDO UN FP NO TIENE WAYPOINTS
        % 
        %     %If color is empty, set the default color to blue
        %     if isempty(color)
        %         color = 'b';
        %     end
        % 
        %     %Display waypoints
        %     plt = plot3([obj.waypoints(:).x], [obj.waypoints(:).y], [obj.waypoints(:).z], 'o', 'MarkerSize', 5, 'MarkerFaceColor', color);
        % 
        %     %Display routes between waypoints
        %     for i = 1:length(obj.waypoints)-1
        %         plot3([obj.waypoints(i).x, obj.waypoints(i+1).x], [obj.waypoints(i).y, obj.waypoints(i+1).y], [obj.waypoints(i).z, obj.waypoints(i+1).z], 'Color', color);
        %     end
        % end
        

        function routeFigure(obj,time_step,color)
            %ROUTEFIGURE This method allow to display the flight plan trajectory
            
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

            %Display trajectory
            tr = obj.trace(time_step);
            plot3(tr(:,2),tr(:,3),tr(:,4), '-', ...
                Color = color );
            %Display waypoints
            plot3([obj.waypoints(:).x], [obj.waypoints(:).y], [obj.waypoints(:).z], 'o',...
                MarkerSize = 5, ...
                MarkerFaceColor = 'w', ...
                MarkerEdgeColor = color );
               
        end

        
        function velocityFigure(obj,time_step,color)
            %VELOCITYFIGURE This method allow to display the flight plan instant velocity
            
            % Check if the flight plan is empty
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

            %Display instant velocity
            tr = obj.trace(time_step);
            plot(tr(:,1),tr(:,5), '-', ...
                Color = color );
            %Display waypoints velocity
            plot([obj.waypoints(:).t], [obj.waypoints(:).v], 'o',...
                MarkerSize = 5, ...
                MarkerFaceColor = 'w', ...
                MarkerEdgeColor = color );
        end


        % function obj = velocityFigure(obj)
        %     %DISPLAYVELOCITY This method allow to display the velocity of the flight plan
        % 
        %     %Check if the flight plan is empty
        %     if isempty(obj.waypoints)
        %         disp('The flight plan is empty');
        %         return
        %     end
        % 
        %     %Find if the figure is already open
        %     fig_name = "Velocity for flight plan ID " + obj.id;
        %     fig = findobj('Type', 'Figure',"Name", fig_name);
        %     if isempty(fig)
        %         %Display a figure with the flight plan
        %         fig = figure("Name", fig_name);
        %         fig.Name = "Velocity for flight plan ID " + obj.id;
        %     else
        %         %Select the figure
        %         figure(fig)
        %         clf(fig)
        %     end
        % 
        %     %Figure settings
        %     hold on;
        %     grid on;
        %     xlabel("Time [s]");
        %     ylabel("Velocity [m/s]");
        %     title("Velocity for flight plan ID " + obj.id);
        % 
        %     %Compute the velocity between waypoints
        %     velocity = [];
        %     for i = 1:length(obj.waypoints)-1
        %         velocity = [velocity, obj.waypoints(i).velocityTo(obj.waypoints(i+1))];
        %     end
        % 
        %     %Display velocity
        %     plot([obj.waypoints(2:end).t], velocity);
        % end
        % 
        %   function obj = velocityFigure(obj,time_step,color)
        %     %DISPLAYVELOCITY This method allow to display the velocity of the flight plan
        % 
        %     %Check if the flight plan is empty
        %     if isempty(obj.waypoints)
        %         disp('The flight plan is empty');
        %         return
        %     end
        % 
        %     %Find if the figure is already open
        %     fig_name = "Velocity for flight plan ID " + obj.id;
        %     fig = findobj('Type', 'Figure',"Name", fig_name);
        %     if isempty(fig)
        %         %Display a figure with the flight plan
        %         fig = figure("Name", fig_name);
        %         fig.Name = "Velocity for flight plan ID " + obj.id;
        %     else
        %         %Select the figure
        %         figure(fig)
        %         clf(fig)
        %     end
        % 
        %     %Figure settings
        %     hold on;
        %     grid on;
        %     xlabel("Time [s]");
        %     ylabel("Velocity [m/s]");
        %     title("Velocity for flight plan ID " + obj.id);
        % 
        %     %Compute the velocity between waypoints
        %     velocity = [];
        %     for i = 1:length(obj.waypoints)-1
        %         velocity = [velocity, obj.waypoints(i).velocityTo(obj.waypoints(i+1))];
        %     end
        % 
        %     %Display velocity
        %     plot([obj.waypoints(2:end).t], velocity);
        % end

        
    end
end

