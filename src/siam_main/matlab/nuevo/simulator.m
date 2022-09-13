%Limpiamos las variables del entorno
clear;
addpath("classes\");

%Numero de drones en la simulacion
max_num_drones = 20;
entrada_usuario = 0;        %Numero indicando drones controlados por usuarios existen
simulink_model = "drone_control_tut3_R22a";

%Inicio de la pool de workers, o si ya está iniciada, parada y reinicio
% try
%     parpool(max_num_drones);
% catch
% %     delete(gcp("nocreate"));
% %     parpool(max_num_drones);
%     disp("Pool de workers ya en ejecución");
% end

%Creamos la entidad de planificacion de planes
DroneOperationPlanningEntity = DroneOperationPlanning(simulink_model);

%Creamos dos operadores de vuelo
droneOperator1 = DroneOperator("Amazon", DroneOperationPlanningEntity);
droneOperator2 = DroneOperator("El Corte Inglés", DroneOperationPlanningEntity);

%Registramos los nuevos operadores
DroneOperationPlanningEntity.registerNewOperator(droneOperator1);
DroneOperationPlanningEntity.registerNewOperator(droneOperator2);

%Creamos dos vehiculos y planes de vuelo asociados a cada vehiculo
flightPlans = OperationalPlan.empty;
vehicles = DroneVehicle.empty;

for i=1:1:1
    %Creamos y registramos un vehiculo para el operador 2
    vehicles(i) = droneOperator2.createNewDroneVehicle("DJI");
    flightPlans(i) = droneOperator2.createNewOperationalPlan(vehicles(i));
    droneOperator2.launch(flightPlans(i));
    pause(5);
end
