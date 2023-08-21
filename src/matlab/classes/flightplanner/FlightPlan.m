classdef FlightPlan < handle
    %FLIGHTPLAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id          int32;
        priority    int8; 

        waypoints   Waypoint = Waypoint.empty;     
        mode        InterpolationModes;
    end
    
    methods

        function obj = FlightPlan(id, waypoints)
            %FLIGHTPLAN Construct for FlightPlan class

            obj.id = id;
            obj.priority = 0;

            %Check if the waypoints list is empty ([])
            if ~isempty(waypoints)
                %Add waypoints to the flight plan
                for i = 1:length(waypoints)
                    obj.setWaypoint(waypoints(i));
                end
            end
            obj.mode = "TP";
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

            switch obj.mode
                case InterpolationModes.TP
                    wp3 = wp1.interpolationTP(wp2,t);
                case InterpolationModes.TPV
                    wp3 = wp1.interpolationTPV(wp2,t);
            end
            p = wp3.position;
     
        end


        function tr = trace(obj, time_step)
            %TRACE This method expands the flight plan behavior over time
            
            instants = obj.init_time : time_step : obj.finish_time;
            tr = zeros(length(instants),7);
            tr(:,1) = instants;
           
            %Get position in time instants
            for i = 1:length(instants)
                p = obj.positionAtTime(tr(i,1));
                tr(i,2:4) = p;
            end

            %Get velocity in time instants
            for i = 1:length(instants)-1
                tr(i,5:7) = (tr(i+1,2:4) - tr(i,2:4)) / time_step;
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


        function routeFigure(obj,time_step,color)
            %ROUTEFIGURE This method allow to display the flight plan trajectory
            
            %Check if the flight plan is empty
            if isempty(obj.waypoints)
                disp('The flight plan is empty');
                return
            end

            %Find if the figure is already open
            fig_name = "FP" + obj.id + ": POSITION";
            fig = findobj("Name", fig_name);
            if isempty(fig)
                %Display a figure with the flight plan
                fig = figure("Name", fig_name);
                fig.Position(3:4) = [800 350];
                fig.NumberTitle = "off";
                fig.MenuBar = "none";
                fig.ToolBar = "none";

            else
                %Select the figure
                figure(fig)
                clf(fig)
            end

            %Figure settings
            tl = tiledlayout(3,5);
            tl.Padding = 'compact';
            tl.TileSpacing = 'tight';
            

            %Display Position 3D
            XYZtile = nexttile([3,3]);
            title("Position 3D")
            hold on
            grid on
            axis equal
            % view(30,10)
            xlabel("x [m]")
            ylabel("y [m]")
            zlabel("z [m]")

            tr = obj.trace(time_step);
            pt = plot3(tr(:,2),tr(:,3),tr(:,4), '-', ...
                Color = color );
            plot3([obj.waypoints(:).x], [obj.waypoints(:).y], [obj.waypoints(:).z], 'o',...
                MarkerSize = 5, ...
                MarkerFaceColor = 'w', ...
                MarkerEdgeColor = color );
               
            %Display Position X versus time
            Xtile   = nexttile([1,2]);
            title("Position versus time");
            hold on
            grid on
            % xlabel("t [s]");
            ylabel("x [m]");

            plot(tr(:,1),tr(:,2), '-', ...
                Color = color );
            plot([obj.waypoints(:).t], [obj.waypoints(:).x], 'o',...
                MarkerSize = 5, ...
                MarkerFaceColor = 'w', ...
                MarkerEdgeColor = color );

            %Display Position Y versus time
            Ytile   = nexttile([1,2]);
            % title("Position Y");
            hold on
            grid on
            % xlabel("t [s]");
            ylabel("y [m]");

            plot(tr(:,1),tr(:,3), '-', ...
                Color = color );
            plot([obj.waypoints(:).t], [obj.waypoints(:).y], 'o',...
                MarkerSize = 5, ...
                MarkerFaceColor = 'w', ...
                MarkerEdgeColor = color );

            %Display Position Z versus time
            Ztile   = nexttile([1,2]);
            % title("Position Z");
            hold on
            grid on
            xlabel("t [s]");
            ylabel("z [m]");

            plot(tr(:,1),tr(:,4), '-', ...
                Color = color );
            plot([obj.waypoints(:).t], [obj.waypoints(:).z], 'o',...
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
            fig_name = "FP" + obj.id + ": VELOCITY";
            fig = findobj('Type', 'Figure',"Name", fig_name);
            if isempty(fig)
                %Display a figure with the flight plan
                fig = figure("Name", fig_name);
                fig.Position(3:4) = [290 455];
                fig.NumberTitle = "off";
                fig.MenuBar = "none";
                fig.ToolBar = "none";
            else
                %Select the figure
                figure(fig)
                clf(fig)
            end

            %Figure settings
            tl = tiledlayout(4,2);
            tl.Padding = 'compact';
            tl.TileSpacing = 'tight';

            %Display instant velocity
            Vtile = nexttile([1,2]);
            title("Velocity versus time")
            hold on
            grid on
            ylabel("3D [m/s]");

            tr = obj.trace(time_step);
            plot(tr(:,1),sqrt(tr(:,5).^2 + tr(:,6).^2 + tr(:,7).^2), '-', ...
                Color = color );

            %Display Velocity X versus time
            Xtile   = nexttile([1,2]);
            hold on
            grid on
            ylabel("vx [m/s]");

            plot(tr(:,1),tr(:,5), '-', ...
                Color = color );
            if obj.mode == InterpolationModes.TPV
                plot([obj.waypoints(:).t], [obj.waypoints(:).vx], 'o',...
                    MarkerSize = 5, ...
                    MarkerFaceColor = 'w', ...
                    MarkerEdgeColor = color );
            end

            %Display Position Y versus time
            Ytile   = nexttile([1,2]);
            hold on
            grid on
            ylabel("vy [m/s]");

            plot(tr(:,1),tr(:,6), '-', ...
                Color = color );
            if obj.mode == InterpolationModes.TPV
                plot([obj.waypoints(:).t], [obj.waypoints(:).vy], 'o',...
                    MarkerSize = 5, ...
                    MarkerFaceColor = 'w', ...
                    MarkerEdgeColor = color );
            end

            %Display Position Z versus time
            Ztile   = nexttile([1,2]);
            hold on
            grid on
            ylabel("vz [m/s]");
            xlabel("t [s]");

            plot(tr(:,1),tr(:,7), '-', ...
                Color = color );
            if obj.mode == InterpolationModes.TPV
                plot([obj.waypoints(:).t], [obj.waypoints(:).vz], 'o',...
                    MarkerSize = 5, ...
                    MarkerFaceColor = 'w', ...
                    MarkerEdgeColor = color );
            end
        end
        

        function distanceFigure(fp1,fp2,time_step,color)
            % Relative distance between two flightplans
            
            %Find if the figure is already open
            fig_name = "FP" + fp1.id + "-FP" + fp2.id + ": DISTANCE";
            fig = findobj('Type', 'Figure',"Name", fig_name);
            if isempty(fig)
                fig = figure("Name", fig_name);
                fig.Position(3:4) = [350 140];
                fig.NumberTitle = "off";
                fig.MenuBar = "none";
                fig.ToolBar = "none";
            else
                figure(fig)
                clf(fig)
            end

            %Figure settings
            ylabel("distance [m]")
            xlabel("time [s]")
            grid on
            hold on
            
            dist = fp1.distanceTo(fp2, time_step);
            plot(dist(:,1),dist(:,2), Color = color );            
        end


    end
end
