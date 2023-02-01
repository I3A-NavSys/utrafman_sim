%Operator class represents an operator of the drone system. Could be a company or a person. Each operator has a unique ID and a name. Each operator has a drone garage where it stores the drones it owns.

classdef Operator < handle
    properties
        operatorId uint32                %Unique operator ID
        operatorName string              %Operator name
        droneGarage = Drone.empty;       %Array of drone objects references
    end
    
    methods
        %Class constructor
        function obj = Operator(operatorName)
            obj.operatorName = operatorName;
        end

        %Register a new drone to the operator adding it to the drone garage
        function obj = regNewDrone(obj, drone)
            drone.operator = obj;
            obj.droneGarage(end+1) = drone;
        end
    end
end

