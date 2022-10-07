%Added classes path
addpath("classes\");

%Creacion de la entidad central del vuelo
UTM = UTMAirspace();

%Anadimos un operador
operator = Operator('Jesus');
UTM.S_Registry.regNewOperator(operator);

%New drone creation
drone = Drone('DJI Phantom');
operator.regNewDrone(drone);
UTM.S_Registry.regNewDrone(drone);

%New flight plan
fp = FlightPlan(0, 1, 2);
UTM.S_Registry.regNewFlightPlan(fp);