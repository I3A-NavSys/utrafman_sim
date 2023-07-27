% Create 10 random waypoints in the map with x,y,z between 0 and 10
way_locs = 10*rand(10,3);

waypoints = Waypoint.empty;
t = 0;

%Create waypoint object
for i = 1:size(way_locs,1)
    waypoints(i) = Waypoint(...
        way_locs(i,1),...
        way_locs(i,2),...
        way_locs(i,3),... 
        t,...
        0,...
        true);
    t = t + 10;
end

fp = FlightPlan(waypoints, 0, 0);
fp.displayRoute()
fp.displayVelocity()

fp.reverseWaypoints()
fp.displayVelocity()

fp.normalizeVelocity()
fp.displayVelocity()