classdef FlightPlanSet < handle
    
    properties
        id          int32      = 0;
        flightplans FlightPlan = FlightPlan.empty;
    end

    methods

        function obj = FlightPlanSet()
            %FLIGHTPLANSET Construct for FlightPlanSet class
        end


        function addFlightPlan(obj, flightplan)
            %ADDFLIGHTPLAN Add a flightplan to the set
            % Insert a flightplan into the set ordenated using the flightplan init_time property

            %Check if the set is empty
            if isempty(obj.flightplans)
                obj.flightplans = flightplan;
            else
                for i = 1:length(obj.flightplans)
                    if obj.flightplans(i).init_time > flightplan.init_time
                        obj.flightplans = [obj.flightplans(1:i-1) flightplan obj.flightplans(i:end)];
                        return
                    end
                end
                obj.flightplans = [obj.flightplans flightplan];
            end

        end


        function removeFlightPlan(obj, id)
            %REMOVEFLIGHTPLAN Remove a flightplan from the set

            %Check if the set is empty
            if isempty(obj.flightplans)
                return
            else
                for i = 1:length(obj.flightplans)
                    if obj.flightplans(i).id == id
                        obj.flightplans = [obj.flightplans(1:i-1) obj.flightplans(i+1:end)];
                        return
                    end
                end
            end
        end

 
        function obj = routesFigure(obj,time_step)
            %ROUTESFIGURE Plot the routes of the flightplans in the set

            %Check if the set is empty
            if isempty(obj.flightplans)
                disp('The set is empty');
                return
            end

            %Find the figure if it exists or create it
            fig_name = "Flight Plan Set ID " + obj.id;
            fig = findobj('Type', 'Figure', 'Name', fig_name);
            if isempty(fig)
                fig = figure('Name', fig_name);
            else
                figure(fig);
                clf(fig);
            end

            %Figure settings
            hold on;
            grid on;
            axis equal;
            view(30,10)
            xlabel("x [m]");
            ylabel("y [m]");
            zlabel("z [m]");
            title("Routes for the set " + obj.id);

            plt = [];
            plt_names = {};
            %Plot the routes
            for i = 1:length(obj.flightplans)
                %Generate random color
                color = rand(1,3);

                fp = obj.flightplans(i);

                %Display trajectory
                tr = fp.trace(time_step);
                plot3(tr(:,2),tr(:,3),tr(:,4), '-', ...
                    Color = color );
                %Display waypoints
                plt(i) = plot3([fp.waypoints(:).x], [fp.waypoints(:).y], [fp.waypoints(:).z], 'o',...
                    MarkerSize = 5, ...
                    MarkerFaceColor = 'w', ...
                    MarkerEdgeColor = color );

                name = "FP ID " + int2str(obj.flightplans(i).id);
                plt_names(i) = {name};
            end

            legend(plt, plt_names);
        end


        function conflicts = detectConflicts(obj, conf_dist, time_step)
            %CONFLICTSDETECTOR Detect conflicts between flightplans
            conflicts = zeros(1e6, 4);
            conflicts_index = 1;
            %Check if the set is empty
            if isempty(obj.flightplans)
                disp('The set is empty');
                return
            end

            %Search min and max time of flightplans in the set
            min_t = inf;
            max_t = 0;

            for i = 1:length(obj.flightplans)
                if obj.flightplans(i).init_time < min_t
                    min_t = obj.flightplans(i).init_time;
                end
                if obj.flightplans(i).finish_time > max_t
                    max_t = obj.flightplans(i).finish_time;
                end
            end

            %Loop to find conflicts
            for t = min_t:time_step:max_t
                tic
                %Get the position of the flightplans at time t using the abstraction layer
                pos_mat = arrayfun(@(fp) fp.positionAtTime(t), obj.flightplans, 'UniformOutput', false);
                %Convert the cell array to a matrix
                pos_mat = cell2mat(pos_mat);
                %Reshape the matrix to a Nx3 matrix with X, Y and Z coordinates
                pos_mat = reshape(pos_mat, 3, length(obj.flightplans))';

                %Calculate the distance between all the flightplans

                for i = 1:length(obj.flightplans)
                    for j = i+1:length(obj.flightplans)
                        dist = norm(pos_mat(i,:) - pos_mat(j,:));
                        % If the distance is less than the conflict distance, add the conflict to the list
                        if dist < conf_dist
                            %conflicts = [conflicts; i j dist t];
                            conflicts(conflicts_index, :) = [i j dist t];
                            conflicts_index = conflicts_index + 1;
                        end
                    end
                end
            end
            conflicts = conflicts(1:conflicts_index-1, :);
        end


        % function conflicts =  detectConflictsBetTimes(obj, conf_dist, time_step, init_time, finish_time)
        %     %CONFLICTSDETECTOR Detect conflicts between flightplans
        %     % Allows to detect conflicts between flightplans in a time interval
        % 
        %     %Get flightplans in the time interval
        %     flightplans = obj.getFlightPlansBetTimes(init_time, finish_time);
        % 
        %     %Construct the FlightPlanSubset
        %     flightplan_subset = FlightPlanSet();
        %     flightplan_subset.id = obj.id;
        %     for i = 1:length(flightplans)
        %         flightplan_subset.addFlightPlan(flightplans(i));
        %     end
        % 
        %     %Detect conflicts
        %     conflicts = flightplan_subset.detectConflicts(conf_dist, time_step);
        % end


        % function flightplans = getFlightPlansBetTimes(obj, init_time, finish_time)
        %     %GETFLIGHTPLANSBETTIMES Get the flightplans in a time interval
        % 
        %     flightplans = [];
        % 
        %     %Filter the flightplans in the time interval
        %     for i = 1:length(obj.flightplans)
        %         if  ~(obj.flightplans(i).init_time >= finish_time || obj.flightplans(i).finish_time <= init_time)
        %             flightplans = [flightplans obj.flightplans(i)];
        %         end
        %     end
        % end

    
    end
end