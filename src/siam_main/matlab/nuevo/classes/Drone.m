classdef Drone < handle

    properties
        droneId
        model
        operator

        flightPlan
    end
    
    methods
        function obj = Drone(model)
            obj.model = model;
        end

        function obj = assignFlightPlan(fp)
            obj.flightPlan = fp;
        end
    end

end

