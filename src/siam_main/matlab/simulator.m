%PRE-SIMULATION TASKs
timer; stop(timerfind); delete(timerfind)  %Stop all timers
addpath("classes\"); %Added classes path
addpath("simulinks\"); %Added classes path
try
    pool = parpool;
catch
end
cancelAll(pool.FevalQueue);
%-----------------------------------------

%Creacion de la entidad central del vuelo
UTM = UTMAirspace();

%Anadimos un operador y lo registramos
operator = Operator('Jesus');
UTM.S_Registry.regNewOperator(operator);

%Registramos los drones, los anade a Gazebo
numDrones = 3;
drone = Drone.empty(0,5);
for i=1:numDrones
    drone(i) = Drone(UTM,'DJI Phantom', [i i 1]);
    operator.regNewDrone(drone(i));
    UTM.S_Registry.regNewDrone(drone(i));
end

UTM.LaunchSimulinksModels();

delay = 0;
fp = FlightPlan.empty(0,numDrones*10);
for i=1:numDrones*10
    route = FlightPlan.GenerateRandomRoute(randi([3 6],1));
    fp(i) = FlightPlan(operator, drone(mod(i,numDrones)+1), drone(mod(i,numDrones)+1).initLoc, route(end), route, delay+60);
    UTM.S_Registry.regNewFlightPlan(fp(i));
    delay = delay + 30/numDrones;
end
