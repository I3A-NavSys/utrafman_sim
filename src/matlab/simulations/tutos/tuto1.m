%Very simple simulation, where num_uavs UAV are allocated and each one
%perform a FP of 500 meters inside 'generated_city' world.

%PRE-SIMULATION TASKs
clc; clear

run('../../tools/UTRAFMAN_init');

timer; stop(timerfind); delete(timerfind);           %Stop all timers


%-----------------------------------------

%World definition file
world = WorldModel(fullfile(UTRAFMAN_DIR,'/gazebo-ros/src/utrafman/worlds/generated_city.wc'));


%Creation of the entire airspace
UTM = UTMAirspace();

%Registry of a new operator
operator = Operator('Sample_Operator');

%FPs
fp = FlightPlanProperties.empty();                %FlightPlan instance
uav = UAVProperties.empty;

%Finish last FP time
last_fp_finish_time = 0;

%Generate a new route (2D)
route = world.getRoute(500, 0);

%Set UAV init pos at the route start
pos = [route(1,:) 3];

%Create and register a new UAV
uav = operator.regNewDrone("dji", pos);
route_3d = zeros(0,3);

%Convert route (2D) to 3D adding random altitude
for x=1:size(route,1)
    route_3d(x,:) = [route(x,:) randi([30 40])];
end

%Uplan generation
fp = FlightPlanProperties(operator, ...          %Operator
                   uav, ...                     %UAV asignation
                   route_3d, ...                     %Route
                   UTM.Gclock+5);       %DTTO (desired time to take off)

%FP registration
operator.regNewFP(fp);

%Sent FP to UAV
operator.sendFlightPlan(fp);

%Set last fp finish time
UTM.setFinishSimulationTime(fp.route(end).T.Sec)


