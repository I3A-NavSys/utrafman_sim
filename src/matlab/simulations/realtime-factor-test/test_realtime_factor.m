%PRE-SIMULATION TASKs
timer; stop(timerfind); delete(timerfind)           %Stop all timers
addpath("./classes/");                              %Added classes path

times = 50;
seconds_per_exec = 10;
n_data_per_exec = seconds_per_exec*4;
data = zeros(times*n_data_per_exec, 5);

for i=1:times
    for k = (i-1)*n_data_per_exec+1: (i-1)*n_data_per_exec+n_data_per_exec
        data(k, 1) = i*10;
    end
end

%-----------------------------------------

%Create a parallel pool to run services in parallel using workers (disabled
%by default)
%parallelpool = gcp;

%World definition file
world = WorldModel('../gazebo-ros/src/utrafman_main/worlds/generated_city.wc');

%Creation of the entire airspace
UTM = UTMAirspace();

%Registry of a new operator
operator = Operator('Sample_Operator', UTM.rosMasterIp);

%UAVs
numUAV = 10;
uavs = UAVProperties.empty;
tbp = 0;                                                    %Delay between UAVs flightplans

%FPs
fp = FlightPlanProperties.empty(0,numUAV*times);                %FlightPlan instance

% %Wait until Gazebo clock has a value
while(UTM.Gclock == -1)
    pause(0.1)
end

routeDistance = 4000;
nextExecution = 0;
simulationTime = 24*60*60;

%Foreach UAV
for i=1:numUAV*times
    %Generate a new route (2D)
    route = world.getRoute(routeDistance, 0);
    %Set UAV init pos at the route start
    pos = [route(1,:) 3];
    %Create and register a new UAV
    uavs(i) = operator.regNewDrone("dji", pos);
    route3d = zeros(0,3);
    %Convert route (2D) to 3D adding random altitude
    for x=1:size(route,1)
        route3d(x,:) = [route(x,:) randi([30 40])];
    end
    
    %Uplan generation
    fp(i) = FlightPlanProperties(operator, ...          %Operator
                       uavs(i), ...                     %UAV asignation
                       route3d, ...                     %Route
                       UTM.Gclock+5+((i-1)*tbp));       %DTTO (desired time to take off)
    
    %FP registration
    operator.regNewFP(fp(i));

    %Sent FP to UAV
    operator.sendFlightPlan(fp(i));
    uavs(i).init_loc = [route3d(end,1:2) 1];
    
    if mod(i,10) == 0
        pause(10);
        [status, cmdout] = system('env -i sh ./simulations/realtime-factor-test/obtain-realtime-factor.sh');
        pause(1);
        d = readmatrix('out.csv');
        data(((i/10)-1)*n_data_per_exec+1:((i/10)-1)*n_data_per_exec+n_data_per_exec, 2:5) = d(1:n_data_per_exec,:);
    end
end
