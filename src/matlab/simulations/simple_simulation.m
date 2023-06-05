%PRE-SIMULATION TASKs
timer; stop(timerfind); delete(timerfind)           %Stop all timers
addpath("./classes/");                              %Added classes path

%-----------------------------------------

%Create a parallel pool to run services in parallel using workers (disabled by default)
%parallelpool = gcp;

%World definition file
world = WorldModel('../gazebo-ros/src/utrafman_main/worlds/generated_city.wc');

%Creation of the entire airspace
UTM = UTMAirspace();

%Registry of a new operator
operator = Operator('Sample_Operator', UTM.rosMasterIp);

%UAVs
num_uavs = 10;
uavs = UAVProperties.empty;

%FPs
fp = FlightPlanProperties.empty(0,num_uavs*1);                %FlightPlan instance

% %Wait until Gazebo clock has a value
while(UTM.Gclock == -1)
    pause(0.1)
end

%Foreach UAV
for i=1:num_uavs
    %Generate a new route (2D)
    route = world.getRoute(500, 0);

    %Set UAV init pos at the route start
    pos = [route(1,:) 3];

    %Create and register a new UAV
    uavs(i) = operator.regNewDrone("dji", pos);
    route_3d = zeros(0,3);

    %Convert route (2D) to 3D adding random altitude
    for x=1:size(route,1)
        route_3d(x,:) = [route(x,:) randi([30 40])];
    end
    
    %Uplan generation
    fp(i) = FlightPlanProperties(operator, ...          %Operator
                       uavs(i), ...                     %UAV asignation
                       route_3d, ...                     %Route
                       UTM.Gclock+5+(i-1));       %DTTO (desired time to take off)
    
    %FP registration
    operator.regNewFP(fp(i));

    %Sent FP to UAV
    operator.sendFlightPlan(fp(i));
end


