figure()
hold all;
view(3);
leyenda = [];
for i=1:size(drone,2)
    co = sprintf('#%1d%1d0000',(9-i*2),(9-i*2));
    plot3(drone(i).locs(:,2),drone(i).locs(:,3),drone(i).locs(:,4),'Color',co);
    leyenda(1,i) = drone(1,i).droneId;
end
legend(string(leyenda))