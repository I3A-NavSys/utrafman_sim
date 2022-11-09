%PRE-SIMULATION TASKs
timer; stop(timerfind); delete(timerfind)  %Stop all timers
addpath("classes\"); %Added classes path
%addpath("simulinks\"); %Added classes path TBDel
%-----------------------------------------

%Creacion de la entidad central del vuelo
UTM = UTMAirspace();

%Anadimos un operador y lo registramos
operator = Operator('Jesus');
UTM.S_Registry.regNewOperator(operator);

%Creamos drones, lo registramos y los anadimos a Gazebo
numDrones = 30;
drone = Drone.empty(0,numDrones);
p = 1;
for i=1:numDrones
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

    drone(i) = Drone(UTM,'DJI Phantom', pos); %Drone instance
    operator.regNewDrone(drone(i)); %Drone registration with the operator
    UTM.S_Registry.regNewDrone(drone(i)); %Drone registration in Uspace
end

delay = 0;
tbp = 0;
fp = FlightPlan.empty(0,numDrones*1);
for i=1:numDrones*1
    %Random route generation
    route = FlightPlan.GenerateRandomRoute(randi([3 6],1));
    %Uplan generation
    fp(i) = FlightPlan(operator, ... %Operator
                       drone(i), ... %Drone asignation
                       route, ... %Route
                       40+((i-1)*tbp)); %DTTO
    %Uplan registration
    UTM.S_Registry.regNewFlightPlan(fp(i));

    delay = delay + 30/numDrones;
end
