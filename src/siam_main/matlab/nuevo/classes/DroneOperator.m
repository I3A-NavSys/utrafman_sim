classdef DroneOperator < handle
    
    properties
        DroneOperatorId
        OperatorName
    
        OperationalPlans = OperationalPlan.empty;
        DroneGarage = DroneVehicle.empty;

        ref_DroneOperationPlanningEntity DroneOperationPlanning
    end
    
    methods
        function obj = DroneOperator(operatorName, dpoe)
            obj.OperatorName = operatorName;
            obj.ref_DroneOperationPlanningEntity = dpoe;
        end

        function newDrone = createNewDroneVehicle(obj, model)
            %Creamos el nuevo drone
            newDrone = DroneVehicle(model);

            %Lo damos de alta en el garaje del operador
            obj.DroneGarage(end+1) = newDrone;

            newDrone.DroneOperator = obj;

            %Lo matriculamos
            obj.ref_DroneOperationPlanningEntity.registerNewVehicle(newDrone);
        end

        function newPlan = createNewOperationalPlan(obj, droneVeh)
            %Nuevo plan de vuelo
            newPlan = OperationalPlan;
            %Propiedades del plan de vuelo
            newPlan.Status = "Created";
            newPlan.DroneOperator = obj;
            newPlan.DroneVehicle = droneVeh;
            newPlan.Base = [1,1,1];
            newPlan.Dest = [1,1,1];
            newPlan.DesiredTakeOffTime = 0;

            %Registramos el plan de vuelo en el operador
            obj.OperationalPlans(end+1) = newPlan;

            %Registramos el plan de vuelo en el planificador de operaciones
            obj.ref_DroneOperationPlanningEntity.registerNewOperationalPlan(newPlan);
        end

        function launch(obj, operationalPlan)
            obj.ref_DroneOperationPlanningEntity.LaunchOperationPlan(obj,operationalPlan);
        end
    end
end

