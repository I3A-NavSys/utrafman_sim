classdef SimulationProcesser < handle
    %This class is used to process the data from the UTM structure and make it easier to process
    %Moreover, it contains functions to get information about the simulation, like telemetry, conflicts, etc.
    
    properties
        S_Registry
        S_Monitoring
        num_uavs
    end
    
    methods
        %This function takes as input the UTM structure and parses it to data structures that are easier to process
        %This is done for performance reasons, since the UTM structure is not optimized for fast processing.

        function obj = SimulationProcesser(UTM)
            tic
            %Parsing S_Registry data (unmodified)
            obj.S_Registry.operators = UTM.S_Registry.operators;
            obj.S_Registry.uavs = UTM.S_Registry.uavs;
            obj.S_Registry.flight_plans = UTM.S_Registry.flight_plans;
            
            %Parsing S_Monitoring data
            obj.num_uavs = length(UTM.S_Registry.uavs);
            obj.S_Monitoring.telemetry = zeros(0,0,0, 'double');
            
            %Read telemetry data and store it in a new matrix
            for i=1:obj.num_uavs
                tel = UTM.S_Monitoring.uavs(i).telemetry;
                %tele = zeros(1,length(tel),16);
                for j=1:length(tel)
                    obj.S_Monitoring.telemetry(i,j,:) = [10e-10*tel(j).Time.Nsec+tel(j).Time.Sec tel(j).Pose.Position.X tel(j).Pose.Position.Y tel(j).Pose.Position.Z tel(j).Pose.Orientation.X tel(j).Pose.Orientation.Y tel(j).Pose.Orientation.Z tel(j).Velocity.Linear.X tel(j).Velocity.Linear.Y tel(j).Velocity.Linear.Z tel(j).Velocity.Angular.X tel(j).Velocity.Angular.Y tel(j).Velocity.Angular.Z double(tel(j).Wip) double(tel(j).Fpip)];
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
        
        %Checks the conflicts between UAVs (v2, fast but not highly accurate, could skip some conflcts, around 5%, depending on telemetry sampling)
        function conflicts = checkConflictsFast(obj,conf_dist)
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
            
                    %Remove those rows with 0 values
                    uav_a = squeeze(locs(i,:,1:4));
                    idx = uav_a(:,1) > 0;
                    uav_a = uav_a(idx,:);
            
                    uav_b = squeeze(locs(j,:,1:4));
                    idx = uav_b(:,1) > 0;
                    uav_b = uav_b(idx,:);
                    
                    %Fixing time to int (for table join)
                    uav_a(:,5) = fix(uav_a(:,1));
                    uav_b(:,5) = fix(uav_b(:,1));
                    
                    %Table generation for join
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
                    
                    %Computing distances between UAVs
                    diff = a_pos - b_pos;
                    dist = [];
                    for o=1:size(diff,1)
                        dist(o) = norm(diff(o,:));
                    end
                    dist = [dist' table_join.TimeInt];

                    %Filtering where distance is less than conflict distance
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

        %Checks the conflicts between UAVs (v1, slow but more accurate than v2)
        function conflicts = checkConflictsComplete(obj,conf_dist)
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
            
                        %Get UAV_j telemetry
                        j_tel = squeeze(locs(j,:,:));
                        idx = j_tel(:,1)>(t_sec-1) & j_tel(:,1)<(t_sec+1);
                        j_tel = j_tel(idx,:);
            
                        %For each telemetry msg in UAV_j
                        for k=1:size(j_tel,1)
                            a = squeeze(t_tel(:,2:4));
                            b = j_tel(k,2:4);
                            
                            %Check if distance is less than conflict distance
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
        function fp = getFpById(obj, id)
            if id == 0
                fp = obj.S_Registry.flight_plans;
                return;
            end
            if id > length(obj.S_Registry.flight_plans)
                fp = 0;
            else
                fp = obj.S_Registry.flight_plans(id);
            end
        end

        % Get UAV object by id
        function uavs = getUavById(obj, id)
            if id == 0
                uavs = obj.S_Registry.uavs;
                return;
            end
            if id > length(obj.S_Registry.uavs)
                uavs = 0;
            else
                uavs = obj.S_Registry.uavs(id);
            end
        end

        % Get operator object by id
        function op = getOperatorById(obj, id)
            if id == 0
                op = obj.S_Registry.operators;
                return;
            end
            if id > length(obj.S_Registry.operators)
                op = 0;
            else
                op = obj.S_Registry.operators(id);
            end
        end

        %Get UAV telemetry table by id
        function tel = getUavTelemetry(obj, uavId)
            tel = obj.S_Monitoring.telemetry(uavId,:,:);
            tel = squeeze(tel);

            %Create a table with telemetry
            tel = array2table(tel, 'VariableNames', cellstr(obj.S_Monitoring.telemetryDef));
        end

        %Filter UAV telemetry table by time
        function tel = filterUavTelemetryByTime(obj, tel, start_t, end_t)
            %Filter by time
            tel = tel(tel.Time > start_t,:);
            tel = tel(tel.Time < end_t,:);
        end

        %Get flightplan waypoints table
        function wp = getFpWaypoints(obj, fp)
            wps = fp.Route;
            times = [wps.T];
            times = [times.Sec] + [times.Nsec]*10e-10;

            wp = table(times', [wps.X]', [wps.Y]', [wps.Z]', [wps.R]', 'VariableNames', {'Time', 'X', 'Y', 'Z', 'R'});
        end

        function fig = telemetryViewer(obj, fpId)
        
            fig = findobj('Type','figure','Name','Telemetry');
            if isempty(fig)
                fig = figure( ...
                    'Name','Telemetry', ...
                    'NumberTitle','off', ...
                    'Position',[200 200 1000 500]);     
            else
                clf(fig);
            end


            fp_obj = obj.getFpById(fpId);
            fp_wps = obj.getFpWaypoints(fp_obj);
            
            uavId = fp_obj.DroneId;
            % uav_obj = obj.getUavById(uavId);
            uav_tel = obj.getUavTelemetry(uavId);
            uav_tel = obj.filterUavTelemetryByTime(uav_tel, min(fp_wps.Time), max(fp_wps.Time));
            
            
            drone.initLoc = [uav_tel.PositionX(1) uav_tel.PositionY(1) uav_tel.PositionZ(1)];
            
            
            %uplan = UTM.S_Registry.flight_plans(fp);
            waypoints = [fp_wps.X, fp_wps.Y, fp_wps.Z];
            
            %Allocating variables
            t       = uav_tel.Time';
            x       = uav_tel.PositionX';
            y       = uav_tel.PositionY';
            z       = uav_tel.PositionZ';
            rotz    = uav_tel.OrientationZ';
            
            dx      = uav_tel.VelLinX';
            dy      = uav_tel.VelLinY';
            dz      = uav_tel.VelLinZ';
            
            %Transform body velocities to world velocities
            eul_trans = zeros(length(dx), 3);
            for i=1:length(dx)
                mat = [cos(-uav_tel.OrientationZ(i)) sin(-uav_tel.OrientationZ(i)) 0 ; -sin(-uav_tel.OrientationZ(i)) -cos(-uav_tel.OrientationZ(i)) 0; 0 0 1];
                eul_trans(i,:) = mat * [dx(i); dy(i); dz(i)]; 
            end
            dx = eul_trans(:,1)';
            dy = eul_trans(:,2)';
            drotz   = uav_tel.VelAngZ';
            
            %Generating data from Uplan
            ut = zeros(1,length(fp_wps.Time)+2);
            
            %Reference data
            %Sim data recoding must be start before Uplan execution and end after Uplan
            %finish. Reference data must start and end at the same time as simulation data to
            %interpolate it correctly.
            
            %Time
            ut(1) = t(1);           %Init telemetry
            ut(2) = fp_wps.Time(1);     %Init Uplan
            for i=1:length(fp_wps.Time)
                ut(i+2) = fp_wps.Time(i);
            end     
            ut(i+3) = ut(i+2)+3;    %Finish uplan
            ut(i+4) = t(end);       %Finish telemetry
            
            %Positions
            ux = [drone.initLoc(1) drone.initLoc(1) fp_wps.X' fp_wps.X(end) fp_wps.X(end)];
            uy = [drone.initLoc(2) drone.initLoc(2) fp_wps.Y' fp_wps.Y(end) fp_wps.Y(end)];
            uz = [0 0 fp_wps.Z' 0 0];
            
            %Generating datetimes of simulation and reference data
            sim_dates = datetime(t,'ConvertFrom','epochtime','Epoch',0);
            ref_dates = datetime(ut,'ConvertFrom','epochtime','Epoch',0);
            
            % X Y Z of the simulation (first load data, then remove duplicates)
            sim_x_timetable = timetable(sim_dates',x');
            sim_x_timetable = retime(sim_x_timetable,unique(sim_x_timetable.Time),'mean');
            sim_y_timetable = timetable(sim_dates',y');
            sim_y_timetable = retime(sim_y_timetable,unique(sim_y_timetable.Time),'mean');
            sim_z_timetable = timetable(sim_dates',z');
            sim_z_timetable = retime(sim_z_timetable,unique(sim_z_timetable.Time),'mean');
            sim_rotz_timetable = timetable(sim_dates',rotz');
            sim_rotz_timetable = retime(sim_rotz_timetable,unique(sim_rotz_timetable.Time),'mean');
            
            % dX dY dZ of the simulation
            sim_dx_timetable = timetable(sim_dates',dx');
            sim_dx_timetable = retime(sim_dx_timetable,unique(sim_dx_timetable.Time),'mean');
            sim_dy_timetable = timetable(sim_dates',dy');
            sim_dy_timetable = retime(sim_dy_timetable,unique(sim_dy_timetable.Time),'mean');
            sim_dz_timetable = timetable(sim_dates',dz');
            sim_dz_timetable = retime(sim_dz_timetable,unique(sim_dz_timetable.Time),'mean');
            sim_drotz_timetable = timetable(sim_dates',drotz');
            sim_drotz_timetable = retime(sim_drotz_timetable,unique(sim_drotz_timetable.Time),'mean');
            
            % X Y Z of the reference (Uplan)
            ref_x_timetable = timetable(ref_dates', ux');
            ref_x_timetable = retime(ref_x_timetable,unique(ref_x_timetable.Time),'mean');
            ref_y_timetable = timetable(ref_dates', uy');
            ref_y_timetable = retime(ref_y_timetable,unique(ref_y_timetable.Time),'mean');
            ref_z_timetable = timetable(ref_dates', uz');
            ref_z_timetable = retime(ref_z_timetable,unique(ref_z_timetable.Time),'mean');
            
            % dX dY dZ of the reference (Uplan)
            %Interpolation of time
            dut = [ut(1):1:ut(end)];
            ref_dates_interpolated = datetime(dut,'ConvertFrom','epochtime','Epoch',0); %Generate time
            ref_dates_temporal = timetable(ref_dates_interpolated');
            
            %Interpolation of XYZ positions
            ref_x_timetable = synchronize(ref_dates_temporal,ref_x_timetable,'first','linear');
            ref_y_timetable = synchronize(ref_dates_temporal,ref_y_timetable,'first','linear');
            ref_z_timetable = synchronize(ref_dates_temporal,ref_z_timetable,'first','linear');
            
            %Generating XYZ velocities
            ref_dx_timetable = timetable(ref_dates_interpolated(1:end-1)', diff(ref_x_timetable.Var1));
            ref_dy_timetable = timetable(ref_dates_interpolated(1:end-1)', diff(ref_y_timetable.Var1));
            ref_dz_timetable = timetable(ref_dates_interpolated(1:end-1)', diff(ref_z_timetable.Var1));
            
            %Syncrionization between tables (sim and ref) and interpolation
            x_timetable = synchronize(sim_x_timetable,ref_x_timetable, 'union', 'linear');
            y_timetable = synchronize(sim_y_timetable,ref_y_timetable, 'union', 'linear');
            z_timetable = synchronize(sim_z_timetable,ref_z_timetable, 'union', 'linear');
            dx_timetable = synchronize(sim_dx_timetable,ref_dx_timetable, 'union', 'linear');
            dy_timetable = synchronize(sim_dy_timetable,ref_dy_timetable, 'union', 'linear');
            dz_timetable = synchronize(sim_dz_timetable,ref_dz_timetable, 'union', 'linear');
            x_timetable = mergevars(x_timetable,[1 2],'NewVariableName','X');
            y_timetable = mergevars(y_timetable,[1 2],'NewVariableName','Y');
            z_timetable = mergevars(z_timetable,[1 2],'NewVariableName','Z');
            dx_timetable = mergevars(dx_timetable,[1 2],'NewVariableName','X');
            dy_timetable = mergevars(dy_timetable,[1 2],'NewVariableName','Y');
            dz_timetable = mergevars(dz_timetable,[1 2],'NewVariableName','Z');
            
            %3D viewer
            subplot(6,2,[1 9]);
            plot3(x,y,z);
            grid on;
            hold on;
            plot3(ux(1:end-1),uy(1:end-1),uz(1:end-1));
            plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),'or');
            legend(["Simulated", "Reference", "Waypoints"],'Location','northwest');
            title("Route 3D view")
            xlabel("pos X");
            ylabel("pos Y");
            zlabel("pos Z");
            
            max_uav = [max(uav_tel.PositionX) max(uav_tel.PositionY) max(uav_tel.PositionZ)];
            min_uav = [min(uav_tel.PositionX) min(uav_tel.PositionY) min(uav_tel.PositionZ)];
            max_fp = [max(fp_wps.X) max(fp_wps.Y) max(fp_wps.Z)];
            min_fp = [min(fp_wps.X) min(fp_wps.Y) min(fp_wps.Z)];
            
            xlim([min(min_uav(1),min_fp(1)) max(max_uav(1),max_fp(1))]);
            ylim([min(min_uav(2),min_fp(2)) max(max_uav(2),max_fp(2))]);
            zlim([min(min_uav(3),min_fp(3)) max(max_uav(3),max_fp(3))]);
            view(45,30);
            hold off;
            
            %Bearing plot
            subplot(6,2,11);
            plot(sim_rotz_timetable, "Var1");
            hold on;
            grid on;
            plot(sim_drotz_timetable, "Var1");
            ylim([-pi,pi]);
            xlabel("Time");
            ylabel("Rot Z");
            legend(["Position", "Velocity"]);
            hold off;
            
            %X,Y, Z pos
            subplot(6,2,[2 6]);
            possp = stackedplot([x_timetable, y_timetable, z_timetable]);
            title("Positions through time");
            xlabel('Simulated time');
            grid on;
            [possp.AxesProperties.LegendLabels] = deal({'Sim','Ref'},{'Sim','Ref'},{'Sim','Ref'});
            [possp.AxesProperties.LegendLocation] = deal('southeast','southeast','southeast');
            
            %X,Y, Z accel
            subplot(6,2,[8 12]);
            possp = stackedplot([dx_timetable, dy_timetable, dz_timetable]);
            title("Velocities through time");
            xlabel('Simulated time');
            grid on;
            [possp.AxesProperties.LegendLabels] = deal({'Sim','Ref'},{'Sim','Ref'},{'Sim','Ref'});
            [possp.AxesProperties.LegendLocation] = deal('southeast','southeast','southeast');
            
            drawnow ;
        end

        %Compute the errors (cum, mean, min, max) in the following of the reference during the simulation
        function error2 = computeFpFollowingError(SP, fp_id)
            %Simulation propierties
                % 1-> Uplan total time (end-init)
                % 2-> Number of Waypoints
                % 3-> Precission (not used)
            simProperties = zeros(1,3);
            
            %Error variable
                % 1-> Total cumulative error
                % 2-> Mean error in each telemetry time
                % 3-> Minimum error
                % 4-> Maximum error
            error2 = zeros(1,4);
            
            %For each U-plan in the entire simulation
            uplans = SP.getFpById(fp_id);
            for j = 1:length(uplans)
                %Take the U-plan
                uplan = uplans(j);
                
                %Take the init and final timie of the U-plan
                t_init = uplan.Dtto;
                t_end = uplan.Route(end).T.Sec;
            
                %U-plan properties
                simProperties(j,1) = t_end-t_init;
                simProperties(j,2) = length(uplan.Route);
            
                %UAV telemetry
                uavTel = SP.filterUavTelemetryByTime(SP.getUavTelemetry(uplan.DroneId), t_init, t_end);
                
                %Max and min
                error2(j,4) = 0;
            
                %For each telemetry sent by the UAV 
                % (precission of the error is determined by the sampling time)
                for i=1:height(uavTel)
                    %Compute difference between U-plan position and simulation UAV position
                    t = uavTel{i, "Time"};
                    reference = FlightPlanProperties.abstractionLayerUplan(uplan, t);
                    real = [uavTel{i,"PositionX"} uavTel{i,"PositionY"} uavTel{i,"PositionZ"}];
                    err = norm(reference-real);
                    %Cumulative error
                    error2(j,1) = error2(j,1) + err;
            
                    %Max and min
                    %Set the first min
                    if i == 1
                        error2(j,3) = norm(reference-real);
                    end
                    
                    %If new min is found
                    if error2(j,3) > err
                        error2(j,3) = err;
                    end
            
                    %If new max is found
                    if error2(j,4) < err
                        error2(j,4) = err;
                    end
                end
            
                error2(j,2) = error2(j,1)/i;
            end
        end

        %Compute the distance traveled by the UAV in a U-plan following
        function dist = computeFpTraveledDistance(obj, fp_id)
            fp = obj.getFpById(fp_id);
            t_init = fp.Dtto;
            t_end = fp.Route(end).T.Sec;
            fp_tel = obj.filterUavTelemetryByTime(obj.getUavTelemetry(fp.DroneId), t_init, t_end);
            fp_tel = [fp_tel{:, "PositionX"} fp_tel{:, "PositionY"} fp_tel{:, "PositionZ"}];
            
            %Compute distance between each point
            dist = 0;
            for i=1:length(fp_tel)-1
                dist = dist + norm(fp_tel(i,:)-fp_tel(i+1,:));
            end
        end
    end
end

