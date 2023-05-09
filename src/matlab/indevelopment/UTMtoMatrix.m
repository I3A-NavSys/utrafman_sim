num_uavs = length(UTM.S_Registry.uavs);
locs = zeros(0,0,0);

%Read telemetry data and store it in a new matrix
for i=1:num_uavs
    tel = UTM.S_Monitoring.uavs(i).telemetry;
    locs_local = zeros(0,0);
    for j=1:length(tel)
        locs(i,j,:) = [tel(j).Pose.Position.X tel(j).Pose.Position.Y tel(j).Pose.Position.Z tel(j).Time.Sec+(tel(j).Time.Nsec*10e-9)];
    end
    %locs(i,:,:) = locs_local;
end

conf_dist = 5;
conflicts = zeros(0,0,0);

%For each UAV
for i=1:num_uavs
    %Get UAV_i telemetry
    i_tel = locs(i,:,:);

    %For each another UAV
    for j=1:num_uavs
        if j <= i
            continue;
        end
        conf_count = 1;

        %For each telemetry msg in UAV_i
        for t=1:length(i_tel)
           t_tel = i_tel(1,t,:);
           t_sec = t_tel(4);

            j_tel = squeeze(locs(j,:,:));
            idx = j_tel(:,4)>(t_sec-1) & j_tel(:,4)<(t_sec+1);
            j_tel = j_tel(idx,:);

            for k=1:size(j_tel,1)
                a = squeeze(t_tel(:,1:3));
                b = j_tel(k,1:3);
                if norm(a-b) <= conf_dist
                    %conflicts_local(end+1) = t_sec;
                    %conflict_row = conflict_row + 1;
                    conflicts(i,j,conf_count) = t_sec;
                    conf_count = conf_count + 1;
                    %fprintf("Conflict find between %d and %d at %f \n", i, j, t_sec);
                    break;
                end
            end
        end
    end
end