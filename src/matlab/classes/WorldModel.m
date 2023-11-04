classdef WorldModel < handle
    %This class is used to model the world and generate random routes
    
    properties
        file        %file object
        filename    %file name
        world_fig   %world figure
        matrix_fig  %matrix figure (to generate routes)

        buildings = zeros(0, 6);
        world_size

        world_occupancy     %matrix of the world occupancy
    end
    
    methods
        %Constructor of the class to read the world from a file
        function obj = WorldModel(file)
            obj.filename = file;

            obj.world_fig = findobj('Type','figure','Name','World');
            if isempty(obj.world_fig)
                obj.world_fig = figure( ...
                    'Name','World', ...
                    'NumberTitle','off', ...
                    'Resize','on');     
            else
                clf(obj.world_fig);
            end

            %obj.matrix_fig = figure();
            obj.readWorldFromFile();
        end

        %Read the world definition from a file (.wc format)
        function readWorldFromFile(obj)
            %Use figure
            figure(obj.world_fig);

            %Open file
            obj.file = fopen(obj.filename, "r");

            %Read world size (first line)
            first_line = fgetl(obj.file);
            world_size_line = split(first_line);
            obj.world_size = str2double(world_size_line);
            
            %Initialize world occupancy matrix (size of the world * 2) with ones
            obj.world_occupancy = ones([obj.world_size(1)*2 obj.world_size(2)*2]);

            %Read buildings
            while ~feof(obj.file)
                %Read building
                file_line = fgetl(obj.file);
                building_line = split(file_line);
                obj.buildings(end+1,:) = str2double(building_line)';
                
                %Add building to the world
                b_size = [obj.buildings(end,4) obj.buildings(end,5) obj.buildings(end,6)];
                start_bound = [(obj.buildings(end,1)-b_size(1)/2) (obj.buildings(end,2)-b_size(2)/2)];
                %end_bound = [(building(1)+b_size(1)/2) (building(2)+b_size(2)/2)]
                
                %Add building to the world occupancy matrix (setting to 0 the cells occupied by the building)
                mat_init = obj.pos2matind(start_bound);
                mat_end = obj.pos2matind(start_bound + b_size(:,1:2)) - [1 1];
                obj.world_occupancy(mat_init(1):mat_end(1), mat_init(2):mat_end(2)) = zeros(b_size(1:2)*2);
            
                %Plot building
                %rectangle('FaceColor', 'red', 'Position',[start_bound,b_size(:,1:2)])
                %plotcube(b_size, obj.buildings(end,1:3)-[b_size(1:2)./2 0], 0, [1,0,0])
                obj.plotBOX(b_size, obj.buildings(end,1:3)-[b_size(1:2)./2 0], 0, [1,0,0])
            end
            %Close file
            fclose(obj.file);

            %Plot world, setting the axis limits, the grid and the 3D view
            xlim([-obj.world_size(1)/2 obj.world_size(1)/2])
            ylim([-obj.world_size(2)/2 obj.world_size(2)/2])
            zlim([0 80])
            xticks(-obj.world_size(1)/2:10:obj.world_size(1)/2)
            yticks(-obj.world_size(2)/2:10:obj.world_size(2)/2)
            grid on
            view(3);
            view(0,90);
        end

        %Function to generate a random route of a given length (in meters) starting from a given position
        function waypoints = getRoute(obj, route_dist, init_loc)
            route_dist = route_dist *2;
            figure(obj.world_fig);
            
            %Get the initial position (if not given, generate a random one)
            if init_loc ~= 0
                x = ceil(init_loc(1));
                y = ceil(init_loc(2));
                pos = obj.pos2matind([x y]);
            else
                while 1
                    pos = obj.randomMatInd();
                    if obj.hasClearAround(pos, 6)
                        break;
                    end
                end
            end

            route = [pos];
            waypoints = [];
            dist = 0;
            finish = 0;
            dir = -1;
        
            %Generate the route
            while ~finish
                %Add waypoint
                waypoints(end+1,:) = obj.matind2pos(pos);
                keep_straight = 1;
                if dir == -1
                    dir = randi([0 3]);
                else
                    randbol = rand < 0.5;
                    if randbol
                        dir = mod(dir+1,4);
                    else
                        dir = mod(dir-1,4);
                    end
                end
                step = obj.getStep(dir);
        
                %Move forward
                while ~finish && keep_straight
                    next_pos = pos + step;
                    %Check if the next position is free
                    if obj.hasClearAround(next_pos, 6)
                        %and if the target position is not occupied by a building
                        if obj.check(next_pos)
                            %Increase the distance
                            dist = dist + 1;
                            pos = next_pos;
                            %pos_real = obj.matind2pos(pos);
                            route(end+1,:) = pos;
                            prob_change_dir = rand();

                            if prob_change_dir < 0.01
                                    keep_straight = 0;
                            end

                            if dist >= route_dist
                                finish = 1;
                            end
                        else
                            keep_straight = 0;
                        end
                    else
                        keep_straight = 0;
                    end
                end
            end
            waypoints(end+1,:) = obj.matind2pos(pos);
            
            %Convert the route to the real world coordinates
            i = 1;
            route_real = [];
            while i <= size(route,1)
                r = obj.matind2pos(route(i,:));
                route_real(end+1,:) = r;
                i = i + 1;
            end

            %Convert the 2D waypoints to 3D waypoints adding a random altitude
            waypoints3D = [];
            for i=1:length(waypoints)
                waypoints3D(i,:) = [waypoints(i,:) randi([30 40])];
            end

            %Plot the route
            hold on;
            color = rand(1,3);
            %line(route_real(:,1)', route_real(:,2)', 'Color', color, 'LineWidth', 1);
            plot3(waypoints3D(:,1)', waypoints3D(:,2)', waypoints3D(:,3)', 'Color',color, 'LineWidth',2);
            drawnow;
        end
    
        %Function to convert a position in the world to the corresponding index in the occupancy matrix
        function ret = pos2matind(obj, pos)
            ret = [pos(1)*2+obj.world_size(1) pos(2)*2+obj.world_size(2)];
        end
        
        %Function to convert a position in the occupancy matrix to the corresponding position in the world
        function ret = matind2pos(obj, pos)
            ret = [(pos(1)/2)-(obj.world_size(1)/2) (pos(2)/2)-(obj.world_size(2)/2)];
        end
        
        %Function to get a random index in the occupancy matrix
        function ret = randomMatInd(obj)
            ret = [randi([50 (obj.world_size(1)*2)-50]) randi([50 (obj.world_size(2)*2)-50])];
        end
        
        %Functio to check if a given position in the occupancy matrix is free (not occupied by a building)
        function ret = check(obj, pos)
            if (pos(1) > size(obj.world_occupancy,2) || pos(1) < 0 || pos(2) > size(obj.world_occupancy,1) || pos(2) < 0)
                ret = 0;
            else 
                ret = obj.world_occupancy(pos(1),pos(2));
            end
        end
        
        %Function to check if the given position has enough free space around it
        function ret = hasClearAround(obj, pos, dist)
            if pos(1) >= 100 && pos(1) <= (obj.world_size(1)*2 - 100) && pos(2) >= 100 && pos(2) <= (obj.world_size(2)*2 - 100) ...
               && obj.check(pos + [0 dist]) && obj.check(pos + [0 -dist]) && obj.check(pos + [dist 0]) && obj.check(pos + [-dist 0]) ...
               && obj.check(pos + [dist dist]) && obj.check(pos + [-dist -dist]) && obj.check(pos + [dist -dist]) && obj.check(pos + [-dist dist])
                ret = 1;
            else
                ret = 0;
            end
        end
        
        %Function to get the step to move in a given direction
        function step = getStep(obj, dir)
            switch dir
                case 0
                    step = [-1 0];
                case 1
                    step = [0 1];
                case 2
                    step = [1 0];
                case 3
                    step = [0 -1];
            end
        end

        function plotBOX(obj,box_size,origin,alpha,color)

            x = box_size(1);
            y = box_size(2);
            z = box_size(3);
            
            BOX.Vertices = [
                0  0  0      % 1
                0  y  0      % 2
                x  y  0      % 3
                x  0  0      % 4
                0  0  z      % 5
                0  y  z      % 6
                x  y  z      % 7
                x  0  z   ]; % 8

            BOX.Vertices(:,1) = BOX.Vertices(:,1) + origin(1);
            BOX.Vertices(:,2) = BOX.Vertices(:,2) + origin(2);
            BOX.Vertices(:,3) = BOX.Vertices(:,3) + origin(3);
                        
            BOX.Faces = [
                1  2  3  4   % bottom face
                5  6  7  8   % top    face
                1  2  6  5   % left   face
                4  3  7  8   % right  face
                1  5  8  4   % front  face
                2  6  7  3 ];% back face
            
            BOX.FaceVertexCData = 0;
            BOX.FaceColor = color;
            BOX.FaceAlpha = alpha;
            BOX.EdgeColor = 'black';
            BOX.LineWidth = 0.1;
            patch(BOX);
            view(3)

        end

    end
end