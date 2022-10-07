classdef Registry < handle

    properties
        %Properties for operators, drones and flightplans.
        operators = Operator.empty;
        operatorLastId = 0;
        
        drones = Drone.empty;
        droneLastId = 0;

        flightPlans = FlightPlan.empty;
        flightPlanLastId = 0;
    end
    
    methods

        function obj = Registry()
        end
        
        function obj = regNewOperator(obj,operator)
            %Compute operatorId
            id = obj.operatorLastId + 1;
            %Assign operatorId
            operator.operatorId = id;
            %Signup in the registry
            obj.operators(id) = operator;
        end

        function obj = regNewDrone(obj, drone)
            %Commpute droneId
            id = obj.droneLastId + 1;
            %Assign droneId
            drone.droneId = id;
            %Signup in the registry
            obj.drones(id) = drone;
        end

        function obj = regNewFlightPlan(obj, fp)
            %Compute flightPlanId
            id = obj.flightPlanLastId + 1;
            %Assign flightPlanLastId
            fp.flightPlanId = id;
            %Signup in the registry
            obj.flightPlans(id) = fp;
        end
    end
end

