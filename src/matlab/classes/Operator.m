classdef Operator < handle
    
    properties
        operatorId uint32                %Unique operator ID
        operatorName string      

        droneGarage = Drone.empty;      %Array of drone objects references
    end
    
    methods
        %Class constructor
        function obj = Operator(operatorName)
            obj.operatorName = operatorName;
        end

        function obj = regNewDrone(obj, drone)
            drone.operator = obj;
            obj.droneGarage(end+1) = drone;
        end
    end
end

