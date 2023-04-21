%PRE-SIMULATION TASKs
timer; stop(timerfind); delete(timerfind)           %Stop all timers
addpath("./classes/");                              %Added classes path

%-----------------------------------------

%Create a parallel pool to run services in parallel using workers (disabled
%by default)
%parallelpool = gcp;

%World definition file
world = WorldModel('../gazebo-ros/src/utrafman_main/worlds/generated_city.wc');

%Creation of the entire airspace
UTM = UTMAirspace();

%ROS variables
rosnode = ros.Node('simulation_executer',UTM.rosMasterIp, 11311);
ros_god_teletransporter = ros.ServiceClient(rosnode,'/godservice/transport_model');

%Registry of a new operator
operator = Operator('Sample_Operator', UTM.rosMasterIp);

%UAVs
numUAV = 20;
uavs = UAVProperties.empty;
tbp = 0;                                                    %Delay between UAVs flightplans

%FPs
fp = FlightPlanProperties.empty(0,numUAV*1);                %FlightPlan instance

% %Wait until Gazebo clock has a value
while(UTM.Gclock == -1)
    pause(0.1)
end

routeDistance = 1000;
nextExecution = 0;
simulationTime = 24*60*60;

%Foreach UAV
for i=1:numUAV
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
end
nextExecution = UTM.Gclock;
round = 1;
while UTM.Gclock < simulationTime
    %Wait to send the next round
    nextExecution = nextExecution + (routeDistance+80)/2 + 10;
    secs = seconds(nextExecution);
    secs.Format = 'hh:mm:ss';
    fprintf('Ronda %d enviada. Proximo envío %s \n', round, secs);
    round = round + 1;
    while (UTM.Gclock < nextExecution)
        pause(0.1);
    end

    %UAV position updater
    for i=1:numUAV
        msg = ros.msggen.utrafman_main.teletransportRequest;
        msg.UavId = i;
        msg.Pose.Position.X = uavs(i).init_loc(1);
        msg.Pose.Position.Y = uavs(i).init_loc(2);
        msg.Pose.Position.Z = uavs(i).init_loc(3);

        if isServerAvailable(ros_god_teletransporter)
            telemetry = call(ros_god_teletransporter,msg,"Timeout",100);
        else
            error("/godservice/transport_model' not available on network");
        end
    end
    

    for i=1:numUAV
        route = world.getRoute(routeDistance, uavs(i).init_loc);
        route3d = zeros(1,3);
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
    end
end

