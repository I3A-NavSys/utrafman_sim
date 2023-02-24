%PRE-SIMULATION TASKs
timer; stop(timerfind); delete(timerfind)       %Stop all timers
addpath("./classes/");                            %Added classes path

%-----------------------------------------

%Creation of the entire airspace
UTM = UTMAirspace();

%Registry of a new operator
operator = Operator('Sample_Operator', UTM.rosMasterIp);

%Drone creation, registry and addition to Gazebo
numUAV = 5;
p = 1;
uavs = UAVProperties.empty;

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
    uavs(i) = operator.regNewDrone("dji", pos);
end

% %For each drone in the U-space, subscribe and init pubs and subs
% % for i=1:numUAV
% %     uavs(i).subToTelemety();                    %Init drone telemetry updates
% %     uavs(i).pubsubToFlightPlan();               %Init drone U-plan publisher
% % end

delay = 0;                                      %Delay between Uplans (in the same drone) (not in use)
tbp = 0;                                        %Delay between drones Uplan
fp = FlightPlanProperties.empty(0,numUAV*1);              %U-plan instance

% %Wait until Gazebo clock has a value
while(UTM.Gclock == -1)
    pause(0.1)
end
 
for i=1:numUAV*1
    rng(i);                                     %Random generator mix
    %Random route generation
    route = FlightPlanProperties.GenerateRandomRoute(randi([6 10],1));
    %Uplan generation
    fp(i) = FlightPlanProperties(operator, ...        %Operator
                       uavs(i), ...         %Drone asignation
                       route, ...           %Route
                       UTM.Gclock+5+((i-1)*tbp));      %DTTO
    
    %FP registration
    operator.regNewFP(fp(i));

    %Sent FP to UAV
    operator.sendFlightPlan(fp(i));
end


