classdef OperationalPlan < handle
    
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


        function t = CheckFinishStatus(obj, timer, time)
            %disp('Ejecutando timer...');
            if obj.BatchsimOutput.State == "finished"
                obj.Status = 'Finished';
                disp("Un drone ya ha terminado");
                stop(obj.FinishTimer);
                %obj.DroneOperator.ref_DroneOperationPlanningEntity.RemoveModelFromGazebo(obj.DroneVehicle);
            end
            t = 1;
        end

        function t = FinishCallback(obj)
        end
    end
end

