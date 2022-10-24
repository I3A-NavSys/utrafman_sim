clear; 
timer; stop(timerfind);    %Stop all timers
addpath("classes\"); %Added classes path

%Creacion de la entidad central del vuelo
UTM = UTMAirspace();

%Anadimos un operador
operator = Operator('Jesus');
UTM.S_Registry.regNewOperator(operator);

%Registramos un nuevo drone
for i=1:1:10
    drone = Drone('DJI Phantom', [i i 1]);
    operator.regNewDrone(drone);
    UTM.S_Registry.regNewDrone(drone);
end

%New flight plan
fp = FlightPlan(operator, drone, [], [], 0);
UTM.S_Registry.regNewFlightPlan(fp);