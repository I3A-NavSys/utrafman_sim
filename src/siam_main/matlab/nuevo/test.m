clear;
addpath("classes\");

%Creamos la entidad de planificacion de planes
DroneOperationPlanningEntity = DroneOperationPlanning;

%Creamos dos operadores de vuelo
droneOperator1 = DroneOperator("Amazon", DroneOperationPlanningEntity);
droneOperator2 = DroneOperator("El Corte Inglés", DroneOperationPlanningEntity);

%Registramos los nuevos operadores
DroneOperationPlanningEntity.registerNewOperator(droneOperator1);
DroneOperationPlanningEntity.registerNewOperator(droneOperator2);

%Creamos y registramos un vehiculo para el operador 2
DroneVehicle1 = droneOperator2.createNewDroneVehicle("DJI");
OperationalPlan1 = droneOperator2.createNewOperationalPlan(DroneVehicle1);
