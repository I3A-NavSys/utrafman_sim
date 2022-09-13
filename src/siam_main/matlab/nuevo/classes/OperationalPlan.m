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

        FinishTimer timer
    end
    
    methods
        function obj = OperationalPlan()
        end


        function t = CheckFinishStatus(obj, ~, ~)
            disp('Timer funcionando');
            t = 1;
        end
    end
end

