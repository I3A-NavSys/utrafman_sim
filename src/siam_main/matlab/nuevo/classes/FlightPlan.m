classdef FlightPlan < handle

    properties
        flightPlanId
        status
        priority

        operator
        drone

        base
        dest
        dtto

        route

    end
    
    methods
        function obj = FlightPlan(base, dest, dtto)
            %Properties
            obj.base = base;
            obj.dest = dest;
            obj.dtto = dtto;
        end

    end
end

