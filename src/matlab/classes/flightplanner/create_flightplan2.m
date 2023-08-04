%Clean console and workspace
clc; clear;

time_step = 0.01;

%Create set of flight plans
fps = FlightPlanSet();


fp1  = FlightPlan(1,Waypoint.empty,0);
%             t  x  y  z
way_data = [ 05 00 05 00
             10 00 05 00
             20 00 05 05
             30 10 05 10
             40 10 05 00
             50 10 05 00 ];
    

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
    

% fp.removeWaypointAtTime(8);
% fp.removeWaypointAtTime(5);
% fp.removeWaypointAtTime(0);
% fp.removeWaypointAtTime(10);



% Display route and velocity of FP
% fp.routeFigure(time_step,'b')
% fp.velocityFigure(0.01,'b')


% tr = fp.trace(1);
% plot(tr(:,1),tr(:,2));
% hold on
% grid on
% plot(tr(:,1),tr(:,3));
% plot(tr(:,1),tr(:,4));
% plot(tr(:,1),tr(:,5));


    
%%Add it to the set
fps.addFlightPlan(fp1);



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
    
%Construct fp
fp2 = FlightPlan(2,waypoints,0);
% fp2.routeFigure(time_step,'r')

dist = fp1.distanceTo(fp2, time_step);
%Figure settings
color = 'b';
plt = plot(dist(:,1),dist(:,2), '.-', ...
    'MarkerSize',5, ...
    'MarkerFaceColor',color, ...
    'Color',color );
grid on;
title("Relative distance between flight plans " + fp1.id + " and " + fp2.id);            
ylabel("distance [m]");
xlabel("Time [s]");

            



%%Add it to the set
fps.addFlightPlan(fp2);





% Other methods
% fp.reverseWaypoints()
% fp.normalizeVelocity()

%Display routes in the FPSet
fps.routesFigure(time_step);

%Compute conflits with distance 3m and time_step 1s
tic
fps.detectConflicts(3,1)
toc


