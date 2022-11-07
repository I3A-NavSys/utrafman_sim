figure()
hold all;
view(3);
leyenda = [];

for i=1:size(drone,2)
    x = zeros(1,size(drone(i).locs,2));
    y = zeros(1,size(drone(i).locs,2));
    z = zeros(1,size(drone(i).locs,2));

    for j = 1:size(drone(i).locs,2)
        x(j) = drone(i).locs(j).Pose.Position.X;
        y(j) = drone(i).locs(j).Pose.Position.Y;
        z(j) = drone(i).locs(j).Pose.Position.Z;
    end

    co = sprintf('#%1d%1d0000',(9-i*2),(9-i*2));
    plot3(x,y,z,'Color',co);
    leyenda(1,i) = drone(1,i).droneId;
end
legend(string(leyenda))