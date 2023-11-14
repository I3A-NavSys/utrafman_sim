%Long-term simulation, where num_uavs UAV are allocated and each one
%receives a FP each round, inside 'generated_city' world. Total number of
%rounds depends on total_sim_time and route_dist.

%PRE-SIMULATION TASKs
timer; stop(timerfind); delete(timerfind)           %Stop all timers
addpath("./classes/");                              %Added classes path
global SP;

%-----------------------------------------

%Simulation config
num_uavs = 20;
route_dist = 500;
total_sim_time =24*60*60; %Minimum Simulation Time

%Create a parallel pool to run services in parallel using workers (disabled by default)
%parallelpclcool = gcp;

%World definition file
world = WorldModel('../gazebo-ros/src/utrafman/worlds/generated_city.wc');

%Creation of the entire airspace
UTM = UTMAirspace();

%ROS variables
rosnode = ros.Node('simulation_executer',UTM.rosMasterIp, 11311);
ros_god_teletransporter = ros.ServiceClient(rosnode,'/godservice/transport_model');

%Registry of a new operator
operator = Operator('Sample_Operator', UTM.rosMasterIp);

%FPs and UAVs
fp = FlightPlanProperties.empty(0,num_uavs*1);                %FlightPlan instance
uavs = UAVProperties.empty;

%Foreach UAV
for i=1:num_uavs
    %Generate a new route (2D)
    route = world.getRoute(route_dist, 0);

    %Set UAV init pos at the route start
    pos = [route(1,:) 3];

    %Create and register a new UAV
    uavs(i) = operator.regNewDrone("dji", pos);
    
    %Convert route (2D) to 3D adding random altitude
    route3d = zeros(0,3);
    for x=1:size(route,1)
        route3d(x,:) = [route(x,:) randi([30 40])];
    end
    
    %Uplan generation
    fp(i) = FlightPlanProperties(operator, ...          %Operator
                       uavs(i), ...                     %UAV asignation
                       route3d, ...                     %Route
                       UTM.Gclock+5+(i-1));       %DTTO (desired time to take off)
    
    %FP registration
    operator.regNewFP(fp(i));

    %Sent FP to UAV
    operator.sendFlightPlan(fp(i));
    
    %Set init loc of UAV
    uavs(i).init_loc = [route3d(end,1:2) 1];

    %Set last fp finish time
    UTM.setFinishSimulationTime(fp(i).route(end).T.Sec)
end

%Schedule next execution
next_exec_time = UTM.Gclock;
round = 1;

%Wait until next round
while UTM.Gclock < total_sim_time
    %Compute next round time
    next_exec_time = UTM.finish_simulation_time + 15;

    %Set new finish simulation time
    UTM.setFinishSimulationTime(next_exec_time + 20);

    %Print in the console
    secs = seconds(next_exec_time);
    secs.Format = 'hh:mm:ss';
    fprintf('Ronda %d enviada. Proximo envío %s \n', round, secs);
    round = round + 1;
    
    %Wait until the next round
    while (UTM.Gclock < next_exec_time)
        pause(0.1);
    end

    %UAV transport service request
    for i=1:num_uavs
        msg = ros.msggen.utrafman.teletransportRequest;
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
    
    %Generate new FPs
    for i=1:num_uavs
        %Generate a new route (2D)
        route = world.getRoute(route_dist, uavs(i).init_loc);
        
        %Convert route (2D) to 3D adding random altitude
        route3d = zeros(1,3);
        for x=1:size(route,1)
            route3d(x,:) = [route(x,:) randi([30 40])];
        end

        %Uplan generation
        fp(i) = FlightPlanProperties(operator, ...          %Operator
                           uavs(i), ...                     %UAV asignation
                           route3d, ...                     %Route
                           UTM.Gclock+5+(i-1));       %DTTO (desired time to take off)
        
        %FP registration
        operator.regNewFP(fp(i));
    
        %Sent FP to UAV
        operator.sendFlightPlan(fp(i));

        %Set init loc of UAV
        uavs(i).init_loc = [route3d(end,1:2) 1];

        %Set last fp finish time
        UTM.setFinishSimulationTime(fp(i).route(end).T.Sec)
    end
end

%Save file
% secs = seconds(UTM.Gclock);
% secs.Format = 'hh:mm:ss';
% filename = sprintf("%s-%s", date, secs);
% save(filename, "UTM", "-v7.3");
    