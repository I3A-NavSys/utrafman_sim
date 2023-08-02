%Clean console and workspace
clc; clear;

%Create set of flight plans
fps = FlightPlanSet();
fp  = FlightPlan(1,Waypoint.empty,0);


%             t  x  y  z
way_data = [ 00 00 05 05
             00 00 05 10
             05 00 50 10
             10 10 05 05 ];
    

for i = 1:size(way_data,1)
    wp = Waypoint(...
        way_data(i,1),...
        way_data(i,2),...
        way_data(i,3),... 
        way_data(i,4),... 
        0,...
        true);

    fp.setWaypoint(wp);
end
    
fp.velocityFigure()

% fp.removeWaypointAtTime(8);
fp.removeWaypointAtTime(5);
% fp.removeWaypointAtTime(0);
% fp.removeWaypointAtTime(10);

    
%%Add it to the set
fps.addFlightPlan(fp);

%%
%             t  x  y  z
way_data = [ 00 05 00 06
             10 05 10 06 ];

waypoints = Waypoint.empty;
    
%Create waypoint object
for i = 1:size(way_data,1)
    waypoints(i) = Waypoint(...
        way_data(i,1),...
        way_data(i,2),...
        way_data(i,3),... 
        way_data(i,4),... 
        0,...
        true);
end
    
%Construct fp
fp = FlightPlan(2,waypoints,0);
    
%%Add it to the set
fps.addFlightPlan(fp);



% Display route and velocity of FP
fp.routeFigure()
fp.velocityFigure()

% Other methods
% fp.reverseWaypoints()
% fp.normalizeVelocity()

%Display routes in the FPSet
fps.routesFigure();

%Compute conflits with distance 3m and time_step 1s
tic
fps.detectConflicts(3,1)
toc


