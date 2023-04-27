num_uavs = length(UTM.S_Registry.uavs);
locs = zeros(0,0,0);

for i=1:num_uavs
    tel = UTM.S_Monitoring.uavs(i).telemetry;
    locs_local = zeros(0,0);
    for j=1:length(tel)
        locs(i,j,:) = [tel(i).Pose.Position.X tel(i).Pose.Position.Y tel(i).Pose.Position.Z tel(i).Time.Sec+(tel(i).Time.Nsec*10e-9)];
    end
    %locs(i,:,:) = locs_local;
end

for i=1:num_uavs
    i_tel = locs(i,:,:);

    for j=1:num_uavs
            if j <= i
                continue;
            end

        conflicts_local = [];

        for t=1:length(i_tel)
           t_tel = i_tel(1,t,:);
           t_sec = t_tel(4);

            j_tel = locs(j,:,:);
            j_tel = j_tel(j_tel(:,:,4)>(t_sec-1) && j_tel(:,:,4)<(t_sec+1));

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