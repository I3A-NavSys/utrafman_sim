classdef DroneVehicle < handle

    properties
        DroneVehicleId
        Model
    end
    
    methods
        function obj = DroneVehicle(model)
            obj.Model = model;
        end
    end
end

