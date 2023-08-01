%Clean console and workspace
clc; clear;

%Create set of flight plans
fps = FlightPlanSet();
max_aerospace_size = 10;        %meters

%             t  x  y  z
way_locs = [ 00 00 05 05
             00 00 05 10
             05 00 500 10
             10 10 05 05 ];

waypoints = Waypoint.empty;
    
%Create waypoint object
for i = 1:size(way_locs,1)
    waypoints(i) = Waypoint(...
        way_locs(i,1),...
        way_locs(i,2),...
        way_locs(i,3),... 
        way_locs(i,4),... 
        0,...
        true);
end
    
%Construct fp
fp = FlightPlan(1,waypoints,0);

fp.removeWaypointAtTime(8);
fp.removeWaypointAtTime(5);
fp.removeWaypointAtTime(0);
fp.removeWaypointAtTime(10);

    
%%Add it to the set
fps.addFlightPlan(fp);

%%
%             t  x  y  z
way_locs = [ 00 05 00 06
             10 05 10 06 ];

waypoints = Waypoint.empty;
    
%Create waypoint object
for i = 1:size(way_locs,1)
    waypoints(i) = Waypoint(...
        way_locs(i,1),...
        way_locs(i,2),...
        way_locs(i,3),... 
        way_locs(i,4),... 
        0,...
        true);
end
    
%Construct fp
fp = FlightPlan(1,waypoints,0);
    
%%Add it to the set
fps.addFlightPlan(fp);



% Display route and velocity of FP
% fp.routeFigure()
% fp.velocityFigure()

% Other methods
% fp.reverseWaypoints()
% fp.normalizeVelocity()

%Display routes in the FPSet
fps.routesFigure();

%Compute conflits with distance 1m and time_step 1s
tic
fps.detectConflicts(3,1)
toc


