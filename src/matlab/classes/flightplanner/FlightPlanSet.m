classdef FlightPlanSet < handle
    properties
        id = 0;
        flightplans FlightPlan = FlightPlan.empty;

        % Figures
        fig_routes;
        fig_time;

    end

    methods
        function obj = FlightPlanSet()
            %FLIGHTPLANSET Construct for FlightPlanSet class
        end

        function obj = addFlightPlan(obj, flightplan)
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

        function obj = removeFlightPlan(obj, flightplan)
            %REMOVEFLIGHTPLAN Remove a flightplan from the set

            %Check if the set is empty
            if isempty(obj.flightplans)
                return
            else
                for i = 1:length(obj.flightplans)
                    if obj.flightplans(i).isequal(flightplan)
                        obj.flightplans = [obj.flightplans(1:i-1) obj.flightplans(i+1:end)];
                        return
                    end
                end
            end
        end

        function obj = routesFigure(obj)
            %ROUTESFIGURE Plot the routes of the flightplans in the set

            %Check if the set is empty
            if isempty(obj.flightplans)
                disp('The set is empty');
                return
            end

            %Find the figure if it exists or create it
            fig_name = "Flight Plan Set ID " + obj.id;
            %fig = findobj('Type', 'Figure', 'Name', fig_name);
            if isempty(obj.fig_routes)
                obj.fig_routes = figure('Name', fig_name);
            else
                figure(obj.fig_routes);
                clf(obj.fig_routes);
            end

            %Figure settings
            hold on;
            grid on;
            axis equal;
            xlabel("x [m]");
            ylabel("y [m]");
            zlabel("z [m]");
            title("Routes for the set " + obj.id);

            plt = zeros(1, length(obj.flightplans));
            plt_names = {};
            %Plot the routes
            for i = 1:length(obj.flightplans)
                %Generate random color
                color = rand(1,3);
                %Plot the route
                plt(i) = obj.flightplans(i).plotRoute(color);

                name = "FP ID " + int2str(obj.flightplans(i).id);
                plt_names(i) = {name};
            end

            legend(plt, plt_names);
        end

        function timeDimensionFig(obj)
            %Check if the set is empty
            if isempty(obj.flightplans)
                disp('The set is empty');
                return
            end

            fig_name = "Time Dimension for the set " + obj.id;
            if isempty(obj.fig_time)
                obj.fig_time = uifigure('Name', fig_name);
                g = uigridlayout(obj.fig_time);
                g.RowHeight = {20,'1x',80, 20};
                g.ColumnWidth = {20, '1x', 20};
            else
                % uifigure(obj.fig_time);
                % clf(obj.fig_time);
            end

            %3D axes
            ax = uiaxes(g);
            ax.Layout.Row = 2;
            ax.Layout.Column = 2;
            ax.XLabel.String = 'x [m]';
            ax.YLabel.String = 'y [m]';
            ax.ZLabel.String = 'z [m]';

            % Set limits for X, Y and Z axes to the maximum distance between the flightplans
            % TODO: use flightplan limits
            ax.XLim = [0 10];
            ax.YLim = [0 10];
            ax.ZLim = [0 10];

            hold(ax,"on");
            grid(ax, "on");

            % Slider
            sld = uislider(g);
            sld.Layout.Row = 3;
            sld.Layout.Column = 2;
            sld.Limits = [obj.flightplans(1).init_time obj.flightplans(end).finish_time];
            sld.ValueChangedFcn = @(sld, event) FlightPlanSet.updateTimePlot(sld, event, ax, obj);
        end

        function animateTimeDimensionFig(obj)
            %Check if the set is empty
            if isempty(obj.flightplans)
                disp('The set is empty');
                return
            end

            sld = findobj(obj.fig_time, 'Type', 'UISlider');
            if isempty(sld)
                disp('The time dimension figure is not initialized');
                return
            end

            % Animation
            for t = obj.flightplans(1).init_time:1:obj.flightplans(end).finish_time
                sld.Value = t;
                %Fire the event
                sld.ValueChangedFcn(sld, []);
                drawnow
                pause(0.01);
            end
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
                pos_mat = arrayfun(@(fp) fp.abstractionLayer(t), obj.flightplans, 'UniformOutput', false);
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
                            conflicts(conflicts_index, :) = [i j dist t];
                            conflicts_index = conflicts_index + 1;
                        end
                    end
                end
            end
            conflicts = conflicts(1:conflicts_index-1, :);
        end

        function conflicts =  detectConflictsBetTimes(obj, conf_dist, time_step, init_time, finish_time)
            %CONFLICTSDETECTOR Detect conflicts between flightplans
            % Allows to detect conflicts between flightplans in a time interval

            %Get flightplans in the time interval
            flightplans = obj.getFlightPlansBetTimes(init_time, finish_time);

            %Construct the FlightPlanSubset
            flightplan_subset = FlightPlanSet();
            flightplan_subset.id = obj.id;
            for i = 1:length(flightplans)
                flightplan_subset.addFlightPlan(flightplans(i));
            end

            %Detect conflicts
            conflicts = flightplan_subset.detectConflicts(conf_dist, time_step);
        end

        function flightplans = getFlightPlansBetTimes(obj, init_time, finish_time)
            %GETFLIGHTPLANSBETTIMES Get the flightplans in a time interval

            flightplans = [];

            %Filter the flightplans in the time interval
            for i = 1:length(obj.flightplans)
                if  ~(obj.flightplans(i).init_time >= finish_time || obj.flightplans(i).finish_time <= init_time)
                    flightplans = [flightplans obj.flightplans(i)];
                end
            end
        end
    end

    methods(Static)
        function updateTimePlot(sld, event, ax, obj)
            %Get the time from the slider
            max_t = fix(sld.Value);
            %Clear the axes
            cla(ax);

            %Plot the routes
            for i = 1:length(obj.flightplans)
                obj.flightplans(i).plotRouteBetTimes(ax, obj.flightplans(i).init_time, max_t);
            end
        end
    end
end