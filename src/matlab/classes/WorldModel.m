classdef WorldModel < handle
    %WORLDMODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        file
        filename
        world_fig
        matrix_fig

        buildings = zeros(0, 6);
        world_size

        world_occupancy
    end
    
    methods
        function obj = WorldModel(file)
            %WORLDMODEL Construct an instance of this class
            %   Detailed explanation goes here
            obj.filename = file;
            obj.world_fig = figure();
            %obj.matrix_fig = figure();
            obj.readWorldFromFile();
        end

        function obj = readWorldFromFile(obj)
            figure(obj.world_fig);

            obj.file = fopen(obj.filename, "r");

            first_line = fgetl(obj.file);
            world_size_line = split(first_line);
            obj.world_size = str2double(world_size_line);
            
            obj.world_occupancy = ones([obj.world_size(1)*2 obj.world_size(2)*2]);

            while ~feof(obj.file)
                file_line = fgetl(obj.file);
                building_line = split(file_line);
                obj.buildings(end+1,:) = str2double(building_line)';
                
                b_size = [obj.buildings(end,4) obj.buildings(end,5) obj.buildings(end,6)];
                start_bound = [(obj.buildings(end,1)-b_size(1)/2) (obj.buildings(end,2)-b_size(2)/2)];
                %end_bound = [(building(1)+b_size(1)/2) (building(2)+b_size(2)/2)]
                
                mat_init = obj.pos2matind(start_bound);
                mat_end = obj.pos2matind(start_bound + b_size(:,1:2)) - [1 1];
                obj.world_occupancy(mat_init(1):mat_end(1), mat_init(2):mat_end(2)) = zeros(b_size(1:2)*2);
            
                rectangle('FaceColor', 'red', 'Position',[start_bound,b_size(:,1:2)])
                plotcube(b_size, obj.buildings(end,1:3)-[b_size(1:2)./2 0], 0.05, [1,0,0])
            end
            fclose(obj.file);

            xlim([-obj.world_size(1)/2 obj.world_size(1)/2])
            ylim([-obj.world_size(2)/2 obj.world_size(2)/2])
            zlim([0 80])
            xticks(-obj.world_size(1)/2:10:obj.world_size(1)/2)
            yticks(-obj.world_size(2)/2:10:obj.world_size(2)/2)
            grid on
            view(3);
            view(0,90);
            %figure(obj.matrix_fig)
            %imagesc(obj.world_occupancy)
        end

        function waypoints = getRoute(obj, route_dist, init_loc)
            figure(obj.world_fig);

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
        
            while ~finish
                waypoints(end+1,:) = obj.matind2pos(pos);
                keep_straight = 1;
                if dir == -1
                    dir = randi([0 3]);
                else
                    dir = mod(dir+1,4);
                end
                step = obj.getStep(dir);
        
                while ~finish && keep_straight
                    next_pos = pos + step;
                    if obj.hasClearAround(next_pos, 6)
                        if obj.check(next_pos)
                           dist = dist + 1;
                           if dist >= route_dist
                               finish = 1;
                           end
                           pos = next_pos;
                           pos_real = obj.matind2pos(pos);
                           route(end+1,:) = pos;
        
                           change_change_dir = rand();
                           if change_change_dir < 0.01
                                keep_straight = 0;
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
            
            i = 1;
            route_real = [];
            while i <= size(route,1)
                r = obj.matind2pos(route(i,:));
                route_real(end+1,:) = r;
                i = i + 1;
            end

            waypoints3D = [];
            for i=1:length(waypoints)
                waypoints3D(i,:) = [waypoints(i,:) randi([30 40])];
            end
            hold on;
            color = rand(1,3);
            line(route_real(:,1)', route_real(:,2)', 'Color', color, 'LineWidth', 1);
            plot3(waypoints3D(:,1)', waypoints3D(:,2)', waypoints3D(:,3)', 'Color',color);
            drawnow;
        end
    
        function ret = pos2matind(obj, pos)
            ret = [pos(1)*2+obj.world_size(1) pos(2)*2+obj.world_size(2)];
        end
        
        function ret = matind2pos(obj, pos)
            ret = [(pos(1)/2)-(obj.world_size(1)/2) (pos(2)/2)-(obj.world_size(2)/2)];
        end
        
        function ret = randomMatInd(obj)
            %ret = [randi([0 1200]) randi([0 1200])];
            ret = [randi([50 (obj.world_size(1)*2)-50]) randi([50 (obj.world_size(2)*2)-50])];
        end
        
        function ret = check(obj, pos)
            if (pos(1) > size(obj.world_occupancy,2) || pos(1) < 0 || pos(2) > size(obj.world_occupancy,1) || pos(2) < 0)
                ret = 0;
            else 
                ret = obj.world_occupancy(pos(1),pos(2));
            end
        end
        
        function ret = hasClearAround(obj, pos, dist)
            if pos(1) >= 100 && pos(1) <= (obj.world_size(1)*2 - 100) && pos(2) >= 100 && pos(2) <= (obj.world_size(2)*2 - 100) ...
               && obj.check(pos + [0 dist]) && obj.check(pos + [0 -dist]) && obj.check(pos + [dist 0]) && obj.check(pos + [-dist 0]) ...
               && obj.check(pos + [dist dist]) && obj.check(pos + [-dist -dist]) && obj.check(pos + [dist -dist]) && obj.check(pos + [-dist dist])
                ret = 1;
            else
                ret = 0;
            end
        end
        
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
    end
end