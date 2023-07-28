fps = FlightPlanSet();

for x=1:10
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
    
    fp = FlightPlan(x,waypoints, 0);
    fps.addFlightPlan(fp);
end

% 
% fp.routeFigure()
% fp.velocityFigure()
% 
% fp.reverseWaypoints()
% fp.velocityFigure()
% 
% fp.normalizeVelocity()
% fp.velocityFigure()

%fps.routesFigure()

tic
%fps.detectConflicts(100,1);
fps.detectConflcitsBetTimes(100,1,-10,20);
toc
