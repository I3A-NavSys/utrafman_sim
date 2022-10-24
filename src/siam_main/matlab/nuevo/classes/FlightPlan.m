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
        function obj = FlightPlan(operator, drone, orig, dest, dtto)
            obj.operator = operator;
            obj.drone = drone;
            obj.orig = orig;
            obj.dest = dest;
            obj.dtto = dtto;
        end

    end
end

