classdef SimulationProcesser < handle
    %SIMULATIONPROCESSER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        S_Registry
        S_Monitoring
        num_uavs
    end
    
    methods
        function obj = SimulationProcesser(UTM)
            tic
            %Parsing S_Registry data (unmodified)
            obj.S_Registry.operators = UTM.S_Registry.operators;
            obj.S_Registry.uavs = UTM.S_Registry.uavs;
            obj.S_Registry.flightPlans = UTM.S_Registry.flightPlans;
            
            %Parsing S_Monitoring data
            obj.num_uavs = length(UTM.S_Registry.uavs);
            obj.S_Monitoring.telemetry = zeros(0,0,0, 'double');
            
            %Read telemetry data and store it in a new matrix
            for i=1:obj.num_uavs
                tel = UTM.S_Monitoring.uavs(i).telemetry;
                %tele = zeros(1,length(tel),16);
                for j=1:length(tel)
                    obj.S_Monitoring.telemetry(i,j,:) = [10e-10*tel(j).Time.Nsec+tel(j).Time.Sec tel(j).Pose.Position.X tel(j).Pose.Position.Y tel(j).Pose.Position.Z tel(j).Pose.Orientation.X tel(j).Pose.Orientation.Y tel(j).Pose.Orientation.Z tel(j).Velocity.Linear.X tel(j).Velocity.Linear.Y tel(j).Velocity.Linear.Z tel(j).Velocity.Angular.X tel(j).Velocity.Angular.Y tel(j).Velocity.Angular.Z double(tel(j).Wip) double(tel(j).Fpip)];
%                         [10e-10*tel(j).Time.Nsec + tel(j).Time.Sec ...
%                         tel(j).Pose.Position.X ...
%                         tel(j).Pose.Position.Y ...
%                         tel(j).Pose.Position.Z ...
%                         tel(j).Pose.Orientation.X ...
%                         tel(j).Pose.Orientation.Y ...
%                         tel(j).Pose.Orientation.Z ...
%                         tel(j).Pose.Orientation.W ...
%                         tel(j).Velocity.Linear.X ...
%                         tel(j).Velocity.Linear.Y ...
%                         tel(j).Velocity.Linear.Z ...
%                         tel(j).Velocity.Angular.X ...
%                         tel(j).Velocity.Angular.Y ...
%                         tel(j).Velocity.Angular.Z ...
%                         tel(j).Wip ...
%                         tel(j).Fpip];
                end
            end
               
            %Definition info inside telemetry
            obj.S_Monitoring.telemetryDef = ["Time",...
                "PositionX", "PositionY", "PositionZ", ...
                "OrientationX", "OrientationY", "OrientationZ",...
                "VelLinX", "VelLinY", "VelLinZ", ...
                "VelAngX", "VelAngY", "VelAngZ",...
                "WaypointInProgress", "FPInProgress"];
            
            clear telemetry
            toc
        end
        
        function conflicts = check_conflicts_fast(obj,conf_dist)
            %Conflict checker (v2, fast but not highly accurate, could skip some conflcts, around 5%, depending on telemetry sampling)
            locs = obj.S_Monitoring.telemetry;

            tic
            %Conflicts memory reservation
            conflicts = zeros(10e7,4);
            conf_index = 0;
            for i=1:obj.num_uavs
                %For each another UAV
                for j=i+1:obj.num_uavs
                    %Number of conflict between them
                    conf_count = 0;
            
                    %Filter 0
                    uav_a = squeeze(locs(i,:,1:4));
                    idx = uav_a(:,1) > 0;
                    uav_a = uav_a(idx,:);
            
                    uav_b = squeeze(locs(j,:,1:4));
                    idx = uav_b(:,1) > 0;
                    uav_b = uav_b(idx,:);
                    
                    %Fixing time to int
                    uav_a(:,5) = fix(uav_a(:,1));
                    uav_b(:,5) = fix(uav_b(:,1));
                    
                    %Table generation
                    table_a = table(uav_a(:,5), uav_a(:,1), uav_a(:,2), uav_a(:,3), uav_a(:,4), 'VariableNames',{'TimeInt' 'Time' 'X' 'Y' 'Z'});
                    table_b = table(uav_b(:,5), uav_b(:,1), uav_b(:,2), uav_b(:,3), uav_b(:,4), 'VariableNames',{'TimeInt' 'Time' 'X' 'Y' 'Z'});
                    
                    %Removing rows with the same time (int)
                    [C,ia] = unique(table_a.TimeInt);
                    [C,ia2] = unique(table_b.TimeInt);
                    
                    %Joining tables
                    table_join = innerjoin(table_a(ia,:), table_b(ia2,:), 'Keys','TimeInt');

                    %Getting positions syncronized in time
                    a_pos = [table_join.X_left table_join.Y_left table_join.Z_left];
                    b_pos = [table_join.X_right table_join.Y_right table_join.Z_right];
                    
                    %Computing distances
                    diff = a_pos - b_pos;
                    dist = [];
                    for o=1:size(diff,1)
                        dist(o) = norm(diff(o,:));
                    end
                    dist = [dist' table_join.TimeInt];
                    %Filtering where distance is less than conflict
                    %distance
                    idx = dist(:,1) < conf_dist;
                    dist = dist(idx,:);

                    %Filling conflicts details [UAVi, UAVj, distance, time]
                    for l=1:size(dist,1)
                        conflicts(conf_index+1,:) = [i j dist(l,:)];
                        conf_count = conf_count + 1;
                        conf_index = conf_index + 1;
                    end
                    %Summary of conflicts
                    if conf_count 
                        fprintf("Conflictos totales entre %d y %d son %d \n", i, j, conf_count)
                    end
                end
            end
            toc
            conflicts = conflicts(1:conf_index,:);
        end

        function conflicts = check_conflicts_complete(obj,conf_dist)
            %Conflict checker (v1, slow but accurate)
            locs = obj.S_Monitoring.telemetry;

            tic
            conflicts = zeros(10e7,4);
            conf_index = 0;
            
            %For each UAV
            for i=1:obj.num_uavs
                %Get UAV_i telemetry
                i_tel = locs(i,:,:);
            
                %For each another UAV
                for j=i:obj.num_uavs
                    if j <= i
                        continue;
                    end
            
                    conf_count = 0;
            
                    %For each telemetry msg in UAV_i
                    for t=1:length(i_tel)
                       t_tel = i_tel(1,t,:);
                       t_sec = t_tel(1);
                       if (~t_sec)
                           continue;
                       end
            
                        j_tel = squeeze(locs(j,:,:));
                        idx = j_tel(:,1)>(t_sec-1) & j_tel(:,1)<(t_sec+1);
                        j_tel = j_tel(idx,:);
            
                        for k=1:size(j_tel,1)
                            a = squeeze(t_tel(:,2:4));
                            b = j_tel(k,2:4);
            
                            if norm(a-b) <= conf_dist
                                conflicts(conf_index+1,:) = [i j norm(a-b) t_sec];
                                conf_count = conf_count + 1;
                                conf_index = conf_index + 1;
                                break;
                            end
                        end
                    end
                    %Summary of conflicts
                    if conf_count 
                        fprintf("Conflictos totales entre %d y %d son %d \n", i, j, conf_count)
                    end
                end
            end
            conflicts = conflicts(1:conf_index,:);
            toc
        end

        % Get flightPlan object by id
        function fp = get_fp_by_id(obj, id)
            if id > length(obj.S_Registry.flightPlans)
                fp = 0;
            else
                fp = obj.S_Registry.flightPlans(id);
            end
        end

        % Get UAV object by id
        function fp = get_uav_by_id(obj, id)
            if id > length(obj.S_Registry.uavs)
                fp = 0;
            else
                fp = obj.S_Registry.uavs(id);
            end
        end

        % Get operator object by id
        function fp = get_operator_by_id(obj, id)
            if id > length(obj.S_Registry.operators)
                fp = 0;
            else
                fp = obj.S_Registry.operators(id);
            end
        end

        %Get UAV telemetry table by id
        function tel = get_uav_telemetry(obj, uav)
            tel = obj.S_Monitoring.telemetry(uav.Id,:,:);
            tel = squeeze(tel);

            %Create a table with telemetry
            tel = array2table(tel, 'VariableNames', cellstr(obj.S_Monitoring.telemetryDef));
        end

        %Filter UAV telemetry table by time
        function tel = filter_uav_telemetry_by_time(obj, tel, start_t, end_t)
            %Filter by time
            tel = tel(tel.Time > start_t,:);
            tel = tel(tel.Time < end_t,:);
        end

        %Get flightplan waypoints table
        function wp = get_fp_waypoints(obj, fp)
            wps = fp.Route;
            times = [wps.T];
            times = [times.Sec] + [times.Nsec]*10e-10;

            wp = table(times', [wps.X]', [wps.Y]', [wps.Z]', [wps.R]', 'VariableNames', {'Time', 'X', 'Y', 'Z', 'R'});
        end
    end
end

