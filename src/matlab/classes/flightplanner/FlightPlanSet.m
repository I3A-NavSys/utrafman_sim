classdef FlightPlanSet < handle
    properties
        id = 0;
        flightplans FlightPlan = FlightPlan.empty;
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
                %Plot the route
                plt(i) = obj.flightplans(i).plotRoute(color);

                name = "FP ID " + int2str(obj.flightplans(i).id);
                plt_names(i) = {name};
            end

            legend(plt, plt_names);
        end

    end
end