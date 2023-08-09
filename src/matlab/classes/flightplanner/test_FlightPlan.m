%Clean console and workspace
clc; clear;

time_step = 0.1;

%%

%             t  x  y  z vx vy vz
way_data = [ 05 00 05 00 00 00 00
             10 00 05 00 00 00 00
             20 00 05 05 00 00 01
             30 10 05 10 00 00 -1 
             40 10 05 00 00 00 00 
             50 10 05 00 00 00 00 ];
    
fp1  = FlightPlan(1,Waypoint.empty);
for i = 1:size(way_data,1)

    wp = Waypoint();
    wp.t = way_data(i,1);
    wp.setPosition(way_data(i,2:4));
    fp1.setWaypoint(wp);
end

% Display
fp1.routeFigure(time_step,'b')
fp1.velocityFigure(time_step,'b')

    

%%
%             t  x  y  z
% way_data = [ 00 05 00 00
%              10 05 00 00
%              20 05 00 10
%              30 05 10 10
%              40 05 10 00
%              50 05 10 00 ];
% 
% waypoints = Waypoint.empty;
    
%Create waypoint object
for i = 1:size(way_data,1)
    waypoints(i) = Waypoint();
    waypoints(i).t = way_data(i,1);
    waypoints(i).setPosition(way_data(i,2:4));
    waypoints(i).setVelocity(way_data(i,5:7));
end
    
fp2  = FlightPlan(2,waypoints);
fp2.mode = "TPV";

% Display
fp2.routeFigure(time_step,'r')
fp2.velocityFigure(time_step,'r')



% Relative distance between two flightplans
figure
title("Relative distance between FP" + fp1.id + " and FP" + fp2.id)
ylabel("distance [m]")
xlabel("time [s]")
grid on
hold on

dist = fp1.distanceTo(fp2, 0.1);
plt = plot(dist(:,1),dist(:,2), Color = 'b' );



%%
%Create set of flight plans
fps = FlightPlanSet();
fps.addFlightPlan(fp1);
fps.addFlightPlan(fp2);

%Display routes in the FPSet
% fps.routesFigure(time_step);

%Compute conflits with distance 3m and time_step 1s
% tic
% fps.detectConflicts(3,0.1)
% toc


