%PRE-SIMULATION TASKs
timer; stop(timerfind); delete(timerfind)       %Stop all timers
addpath("./classes/");                            %Added classes path

%-----------------------------------------

%Creation of the entire airspace
UTM = UTMAirspace();

%Registry of a new operator
operator = Operator('Sample_Operator');
UTM.S_Registry.regNewOperator(operator);

%Drone creation, registry and addition to Gazebo
numUAV = 2;
uavs = Drone.empty(0,numUAV);
p = 1;

for i=1:numUAV
    %Generating the spawn position
    q = mod(i,4);
    switch q
        case 0
            p = p+1;
            pos = [p/3 p/3 0.3];
        case 1
            pos = [-p/3 p/3 0.3];
        case 2
            pos = [-p/3 -p/3 0.3];
        case 3
            pos = [p/3 -p/3 0.3];
    end
    uavs(i) = Drone(UTM,'DJI Phantom', pos);    %Drone instance
    operator.regNewDrone(uavs(i));              %Drone registration with the operator
    UTM.S_Registry.regNewDrone(uavs(i));        %Drone registration in the airspace
end

pause(1);

%For each drone in the U-space, subscribe and init pubs and subs
% for i=1:numUAV
%     uavs(i).subToTelemety();                    %Init drone telemetry updates
%     uavs(i).pubsubToFlightPlan();               %Init drone U-plan publisher
% end

delay = 0;                                      %Delay between Uplans (in the same drone) (not in use)
tbp = 0;                                        %Delay between drones Uplan
fp = FlightPlan.empty(0,numUAV*1);              %U-plan instance

%Wait until Gazebo clock has a value
while(UTM.Gclock == -1)
    pause(0.1)
end

%Random route generation
%route = FlightPlan.generateRandomRoute(randi([6 10],1));
route1 = [[0  10 2]; [0  9 2]; [0 -10 2]];
route2 = [[0 -10 2]; [0 -9 2]; [0  10 2]];

% route1 = [[0 0 10]; [0 0 1 ]];
% route2 = [[0 0 1];  [0 0 10]];

%Uplan generation
fp(1) = FlightPlan(operator, ...        %Operator
                   uavs(1), ...         %Drone asignation
                   route1, ...           %Route
                   UTM.Gclock+5);      %DTTO

fp(2) = FlightPlan(operator, ...        %Operator
                   uavs(2), ...         %Drone asignation
                   route2, ...           %Route
                   UTM.Gclock+5);      %DTTO

%Uplan registration
UTM.S_Registry.regNewFlightPlan(fp(1));
UTM.S_Registry.regNewFlightPlan(fp(2));
