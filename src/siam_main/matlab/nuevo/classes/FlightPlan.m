classdef FlightPlan < handle

    properties
        flightPlanId
        status
        priority

        operator
        drone

        orig
        dest
        dtto

        route

    end
    
    methods
        function obj = FlightPlan(base, dest, dtto)
            %Properties
            obj.orig = base;
            obj.dest = dest;
            obj.dtto = dtto;
        end

    end
end

