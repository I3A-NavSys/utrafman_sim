classdef DroneVehicle < handle

    properties
        DroneVehicleId
        Model

        DroneOperator

        SimulationInput
    end
    
    methods
        function obj = DroneVehicle(model)
            obj.Model = model;
        end
    end
end

