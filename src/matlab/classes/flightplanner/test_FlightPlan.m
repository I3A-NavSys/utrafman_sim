%Clean console and workspace
clc; clear;

time_step = 0.01;

%%

%             t  x  y  z
way_data = [ 05 00 05 00
             10 00 05 00
             20 00 05 05
             30 10 05 10
             40 10 05 00
             50 10 05 00 ];
    
fp1  = FlightPlan(1,Waypoint.empty,0);
for i = 1:size(way_data,1)

    wp = Waypoint(...
        way_data(i,1),...
        way_data(i,2),...
        way_data(i,3),... 
        way_data(i,4),... 
        0,...
        true);

    fp1.setWaypoint(wp);
end
    




% % Display route and velocity of FP
% fp1.routeFigure(time_step,'b')
fp1.velocityFigure(time_step,'b')
% 
% tr = fp1.trace(0.01);
% figure
% plot(tr(:,1),tr(:,2));
% hold on
% grid on
% plot(tr(:,1),tr(:,3));
% plot(tr(:,1),tr(:,4));
% plot(tr(:,1),tr(:,5));



% Other methods
% fp1.removeWaypointAtTime(8);
% fp1.reverseWaypoints()
% fp1.normalizeVelocity()


    

%%
%             t  x  y  z
way_data = [ 00 05 00 00
             10 05 00 00
             20 05 00 10
             30 05 10 10
             40 05 10 00
             50 05 10 00 ];

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
    
fp2  = FlightPlan(2,waypoints,0);




% Relative distance between two flightplans


figure
title("Relative distance between flight plans " + fp1.id + " and " + fp2.id)
ylabel("distance [m]")
xlabel("time [s]")
grid on
hold on
            
dist = fp1.distanceTo(fp2, 5);
plt = plot(dist(:,1),dist(:,2),  ...
    LineWidth = 2, ...
    Color = 'r' );

dist = fp1.distanceTo(fp2, 1);
plt = plot(dist(:,1),dist(:,2),  ...
    LineWidth = 2, ...
    Color = 'g' );


dist = fp1.distanceTo(fp2, 0.1);
plt = plot(dist(:,1),dist(:,2),  ...
    LineWidth = 2, ...
    Color = 'b' );




%%
%Create set of flight plans
fps = FlightPlanSet();
fps.addFlightPlan(fp1);
fps.addFlightPlan(fp2);



%Display routes in the FPSet
fps.routesFigure(time_step);

%Compute conflits with distance 3m and time_step 1s
tic
fps.detectConflicts(3,0.1)
toc


