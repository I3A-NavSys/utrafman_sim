%Clean console and workspace
clc; clear;

%Create set of flight plans
fps = FlightPlanSet();

number_of_flightplans = 2;
waypoints_per_fp = 50;
max_aerospace_size = 10;        %meters
    
for x=1:number_of_flightplans
    % Create random waypoints location
    way_locs = max_aerospace_size*rand(waypoints_per_fp,3);
    
    waypoints = Waypoint.empty;
    t = 0;
    
    %Create waypoint object
    for i = 1:size(way_locs,1)
        waypoints(i) = Waypoint(...
            t,...
            way_locs(i,1),...
            way_locs(i,2),...
            way_locs(i,3),... 
            0,...
            true);
        t = t + 10;
    end
    
    %Construct fp
    fp = FlightPlan(x,waypoints, 0);
    %%Add it to the set
    fps.addFlightPlan(fp);
end

% Display route and velocity of FP
% fp.routeFigure()
% fp.velocityFigure()

% Other methods
% fp.reverseWaypoints()
% fp.normalizeVelocity()

%Display routes in the FPSet
fps.routesFigure();

tic
%Comput conflits with distance 1m and time_step 1s
fps.detectConflicts(1,1)
toc

% Other method
%fps.detectConflictsBetTimes(100,0,-10,20);
