%Very simple simulation, where num_uavs UAV are allocated and each one
%perform a FP of 500 meters inside 'generated_city' world.

%PRE-SIMULATION TASKs
clc; clear;
run('../../tools/UTRAFMAN_init');


gz = Gazebo(ROS_MASTER_IP);
gz.pause;
gz.reset;

%-----------------------------------------

% %World definition file
% world = WorldModel(fullfile(UTRAFMAN_DIR,'/gazebo-ros/src/utrafman/worlds/generated_city.wc'));
% 
% %Generate a new route (2D)
% route = world.getRoute(500, 0);
% %Convert route (2D) to 3D adding random altitude
% route_3d = zeros(0,3);
% for x=1:size(route,1)
%     route_3d(x,:) = [route(x,:) randi([30 40])];
% end


route_3d = [

         0     0    10
         0    10    10
        10    10    10
        10     0    10   
         0     0    10
         0     0     0
];

%Set UAV init pos at the route start
pos = [0 0 1];



registrator = S_Registry();
registrator.execute(ROS_MASTER_IP);

% monitor = S_Monitoring();
% monitor.execute(ROS_MASTER_IP);



%Create and register a new UAV
operator = Operator('Sample_Operator');
uav = UAVProperties.empty;
uav = operator.regNewDrone("dji", pos);


%Uplan generation
fp = FlightPlanProperties.empty();                %FlightPlan instance
fp = FlightPlanProperties(operator, ...          %Operator
                   uav, ...                     %UAV asignation
                   route_3d, ...                     %Route
                   gz.Gclock+10);       %DTTO (desired time to take off)

%FP registration
operator.regNewFP(fp);

%Sent FP to UAV
operator.sendFlightPlan(fp);

gz.play;



