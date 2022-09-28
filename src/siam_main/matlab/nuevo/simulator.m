%Limpiamos las variables del entorno
%clear;
addpath("classes\");

%Numero de drones en la simulacion
max_num_drones = 4;
entrada_usuario = 0;        %Numero indicando drones controlados por usuarios existen
simulink_model = "drone_control_tut3_R22a";
zona_spawn_nw = [-5 -5];
zona_spawn_se = [5 5];

%Creamos la entidad de planificacion de planes
DroneOperationPlanningEntity = DroneOperationPlanning(simulink_model, max_num_drones, zona_spawn_nw, zona_spawn_se);

%Creamos dos operadores de vuelo
droneOperator1 = DroneOperator("Amazon", DroneOperationPlanningEntity);
droneOperator2 = DroneOperator("El Corte Inglés", DroneOperationPlanningEntity);

%Registramos los nuevos operadores
DroneOperationPlanningEntity.registerNewOperator(droneOperator1);
DroneOperationPlanningEntity.registerNewOperator(droneOperator2);

%Creamos dos vehiculos y planes de vuelo asociados a cada vehiculo
flightPlans = OperationalPlan.empty;
vehicles = DroneVehicle.empty;

for i=1:1:max_num_drones
    %Creamos y registramos un vehiculo para el operador 2
    vehicles(i) = droneOperator2.createNewDroneVehicle("DJI");
    %flightPlans(i) = droneOperator2.createNewOperationalPlan(vehicles(i));
    %droneOperator2.launch(flightPlans(i));
    pause(0.3);
end

%Puesta en marca de la ejecucion
%DroneOperationPlanningEntity.LaunchSimulation();