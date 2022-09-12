classdef OperationalPlan < handle
    %OPERATIONALPLAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        AssignedId %Asignado por el DroneOperationPlanning
        Status
        Priority   %Asignado por el DroneOperationPlanning

        DroneOperator DroneOperator;
        DroneVehicle DroneVehicle;

        Base
        Dest
        DesiredTakeOffTime

        AssignedRoute
        SimulinkInput
        BatchsimOutput
    end
    
    methods
        function obj = OperationalPlan()
        end
    end
end

