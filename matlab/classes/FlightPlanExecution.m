classdef FlightPlanExecution < handle

    properties
        fligtPlan FlightPlan
        status uint8
    end
    
    methods
        function obj = FlightPlanExecution(base, dest, dtto)
            %Properties
            obj.base = base;
            obj.dest = dest;
            obj.dtto = dtto;
        end

    end
end

