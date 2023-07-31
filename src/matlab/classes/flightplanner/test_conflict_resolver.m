%Clean console and workspace
clc; clear;

%Create set of flight plans
fps = FlightPlanSet();
way_locs = zeros([0,3]);

for x=1:2
    % Create random waypoints location
    for i = 1:5
        way_locs(i,:) = [4+x,    i,    1];
    end
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
%fps.routesFigure();

%Time dimension figure and animation
fps.timeDimensionFig();
fps.animateTimeDimensionFig();

cr = ConflictResolver();
cr.simplerResolve(fps);

fps.timeDimensionFig();
fps.animateTimeDimensionFig();

%Comput conflits with distance 1m and time_step 1s
%fps.detectConflicts(1,1)

% Other method
%fps.detectConflictsBetTimes(100,0,-10,20);
