classdef BuildingGenerator
    properties
        sx = [10, 50]
        sy = [10, 50]
        sz = [10, 50]
    end
    
    methods
        function obj = BuildingGenerator(size_x, size_y, size_z)
            obj.sx = size_x;
            obj.sy = size_y;
            obj.sz = size_z;
        end
        
        % Method to generate a building
        function building = generateBuilding(obj)
            x = randi([obj.sx(1), obj.sx(2)]);
            y = randi([obj.sy(1), obj.sy(2)]);
            z = randi([obj.sz(1), obj.sz(2)]);
            
            building = Building([x, y, z]);
        end
    end
end