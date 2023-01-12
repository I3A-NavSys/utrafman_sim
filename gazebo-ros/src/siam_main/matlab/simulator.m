%PRE-SIMULATION TASKs
timer; stop(timerfind); delete(timerfind)  %Stop all timers
addpath("classes\"); %Added classes path
%-----------------------------------------

%Creacion de la entidad central del vuelo
UTM = UTMAirspace();

%Anadimos un operador y lo registramos
operator = Operator('Jesus');
UTM.S_Registry.regNewOperator(operator);

%Creamos drones, lo registramos y los anadimos a Gazebo
numDrones = 1;
drone = Drone.empty(0,numDrones);
p = 1;
for i=1:numDrones
    %Generamos una posicion de inicio de vuelo
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
    pos = [-34 8 0.2];
    drone(i) = Drone(UTM,'DJI Phantom', pos); %Drone instance
    operator.regNewDrone(drone(i)); %Drone registration with the operator
    UTM.S_Registry.regNewDrone(drone(i)); %Drone registration in Uspace
end

pause(1);

for i=1:numDrones  
    %Init drone telemetry updates
    drone(i).subToTelemety();
    %Init drone Uplan publisher
    drone(i).pubsubToFlightPlan();
end

pause(1);
delay = 0;      %Delay between Uplans (in the same drone) (not in use)
tbp = 0;        %Delay between drones Uplan
fp = FlightPlan.empty(0,numDrones*1); %Uplan instance

for i=1:numDrones*1
    rng(i);     %Random generator mix
    %Random route generation
%     route = FlightPlan.GenerateRandomRoute(randi([6 10],1));
    route = [
          [-34 8 0.2 25]
          [-34 8 12 35]
          [-34 1 12 45]
          [109 1 12 100]
          [109 27 12 120]
          [98 27 12 125]
          [98 27 0 140]
        ];

    %Uplan generation
    fp(i) = FlightPlan(operator, ...    %Operator
                       drone(i), ...    %Drone asignation
                       route, ...       %Route
                       5+((i-1)*tbp)); %DTTO
    %Uplan registration
    UTM.S_Registry.regNewFlightPlan(fp(i));
end
