%Get data
reg_ser = UTM.S_Registry;
fps = reg_ser.flightPlans;
num_uavs = length(reg_ser.uavs);
%Remove last fp for each uav
fps = fps(1:(length(fps)-num_uavs));
%FP distances
fps_distances = zeros(0,num_uavs);

%Obtain distance of each FP
% for i=1:length(fps)
%     uav_id = fps(i).DroneId;
%     tel = get_telemetry_fp(UTM, i);
%     dist = compute_telemetry_dist(tel);
%     row = fix((i-1)/num_uavs)+1;
%     col = mod(i-1,num_uavs)+1;
%     fps_distances(row, col) = dist;
% end

fprintf("Distancia media recorrida por cada UAV: %f \n", mean(mean(fps_distances)));
fprintf("Std distancia recorrida por los UAV: %f \n", std(std(fps_distances)));
%figure()
%bar(1:num_uavs,mean(fps_distances))


%jesus = get_telemetry_uav_time(UTM,1,1000);

% pos = [];
% for i=1:length(jesus)
%     pos(i,1) = jesus(i).Pose.Position.X;
%     pos(i,2) = jesus(i).Pose.Position.Y;
%     pos(i,3) = jesus(i).Pose.Position.Z;
%     pos(i,4) = jesus(i).Time.Sec + (jesus(i).Time.Nsec*10e-9);
% end
%jover = interp3(pos(:,1)',pos(:,2)',pos(:,3)',pos(:,4)',1:10,1:10,1:10)


conf_dist = 50;
n_workers = 10;
parpool(n_workers);
conflicts = zeros(n_workers,num_uavs,0);

parfor i=1:num_uavs
    %conflict_row = 1;
    i_tel = get_telemetry_uav(UTM, i);

    for j=1:num_uavs
            if j <= i
                continue;
            end

        conflicts_local = [];

        for t=1:length(i_tel)
           t_tel = i_tel(t);
           t_sec = t_tel.Time.Sec + (t_tel.Time.Nsec * 10e-9);

            j_tel = get_telemetry_uav_time(UTM,j,t_sec);
            for k=1:length(j_tel)
                a = [t_tel.Pose.Position.X t_tel.Pose.Position.Y t_tel.Pose.Position.Z];
                b = [j_tel(k).Pose.Position.X j_tel(k).Pose.Position.Y j_tel(k).Pose.Position.Z];
                if norm(a-b) <= conf_dist
                    conflicts_local(end+1) = t_sec;
                    %conflict_row = conflict_row + 1;
                    break;
                end
            end
        end
        conflicts(i,j,:) = conflicts_local;
    end
end
fprintf("Terminado");




function tel = get_telemetry_uav(UTM, uav_id)
    mon_ser = UTM.S_Monitoring;
    tel = mon_ser.uavs(uav_id).telemetry;
end

function telemetry = get_telemetry_fp(UTM, fpid)
    reg_ser = UTM.S_Registry;
    mon_ser = UTM.S_Monitoring;

    fp = reg_ser.flightPlans(fpid);
    uavid = fp.DroneId;

    %Getting the init and end of the FlightPlan
    fp_init = fp.Dtto;
    fp_end = fp.Route(end).T.Sec;

    %Getting the index of the telemetry of the FP
    uav_locs = mon_ser.uavs(uavid).telemetry;
    index_init = 0;
    index_end = 0;
    for i=1:length(uav_locs)
        loc_sec = uav_locs(i).Time.Sec;
        %Search init index
        if (index_init == 0)
            if (loc_sec >= fp_init)
                index_init = i;
                continue;
            end
        end
        %Search end index
        if (loc_sec <= fp_end)
            continue;
        else
            index_end = i;
            break;
        end
    end
    %Getting FP telemetry
    telemetry = uav_locs(index_init:index_end);
end

function tel = get_telemetry_uav_time(UTM, uav_id, time)
    mon_ser = UTM.S_Monitoring;
    uav_tel = mon_ser.uavs(uav_id).telemetry;

    %Getting the init and end of the FlightPlan
    tel_init = time-1;
    tel_end = time+1;

    %Getting the index of the telemetry of the FPy;
    index_init = 0;
    index_end = 0;

    for i=1:length(uav_tel)
        loc_time = uav_tel(i).Time.Sec + (uav_tel(i).Time.Nsec*10e-9);
        %Search init index
        if (index_init == 0)
            if (loc_time >= tel_init)
                index_init = i;
                continue;
            end
        end
        %Search end index
        if (loc_time <= tel_end)
            continue;
        else
            index_end = i;
            break;
        end
    end
    %Getting FP telemetry
    if index_init ~= 0 && index_end ~= 0 && index_init ~= index_end
        tel = uav_tel(index_init:index_end);
    else
        tel = [];
    end
end

function distance = compute_telemetry_dist(tel)
    distance = 0;
    for i=2:length(tel)
        first = tel(i-1);
        last = tel(i);
    
        a = [first.Pose.Position.X first.Pose.Position.Y first.Pose.Position.Z];
        b = [last.Pose.Position.X last.Pose.Position.Y last.Pose.Position.Z];
    
        distance = distance + norm(a-b);
    end
end