classdef Operator < handle
    
    properties
        operatorId
        operatorName

        droneGarage = Drone.empty;
    end
    
    methods
        function obj = Operator(operatorName)
            %Save operator name
            obj.operatorName = operatorName;
        end

        function obj = regNewDrone(obj, drone)
            drone.operator = obj;
            %Singup in the garage
            obj.droneGarage(end+1) = drone;
        end
    end
end

