%Entidad que planifica los vuelos en el espacio aéreo, correspondiente con
%el bloque "Drone Operation Planning"

classdef DroneOperationPlanning < handle
    
    properties
        %Registro de operadores y planes de operaciones
        Operators = DroneOperator.empty;
        OperationalPlans = OperationalPlan.empty;

        %Ultimos IDs asignados. Ninguno de los IDs será 0
        lastDroneOperatorId = 0;
        lastOperationalPlanId = 0;
        lastDroneVehicleId = 0;

        %Para conectar con ROS
        rosMasterIp = "192.168.1.129";
        rosMasterPort = 11311;
        rosPublisher;
        rosMessage;

        %Modelos SDF de los drones
        droneModel;
        userDroneModel;
        dronesUsedPositions = zeros(9,9);
        
        %Simulink y bloques de Simulink con los parametros de ROS
        sim_ModelName;
        sim_pub_bus_command;
        sim_sub_camera;
        sim_sub_odometry;
    end
    
    methods
        function obj = DroneOperationPlanning(simulink_model)
            %Obtencion de los modelos de drones para Gazebo
            obj.droneModel = fileread('../../models/dronechallenge_models/drone/model_template_1.sdf');
            obj.userDroneModel = fileread('../../models/dronechallenge_models/drone/model_template_2.sdf');
            
            %Nombre del modelo de Simulink y de los bloques
            obj.sim_ModelName = simulink_model;
            obj.sim_pub_bus_command = obj.sim_ModelName + "/drone simulator/command bus/pub_bus_command";
            obj.sim_sub_camera = obj.sim_ModelName + "/drone simulator/camera bus/ROS communication/sub_camera";
            obj.sim_sub_odometry = obj.sim_ModelName + "/drone simulator/imu bus/sub_odometry";
            open_system(obj.sim_ModelName);

            %Conexion con el master de ROS
            obj.ConnectWithROSMaster();
        end


        %Registro de un nuevo operador
        function obj = registerNewOperator(obj, newOperator)
            obj.lastDroneOperatorId = obj.lastDroneOperatorId + 1;
            newOperator.DroneOperatorId = obj.lastDroneOperatorId;
            obj.Operators(end+1) = newOperator;
        end


        %Registro de un nuevo vehiculo para un operador de vuelo
        function registerNewVehicle(obj, droneVehicle)
            obj.lastDroneVehicleId = obj.lastDroneVehicleId + 1;
            droneVehicle.DroneVehicleId = obj.lastDroneVehicleId;
        end


        %Registro de un nuevo plan de operacion para un operador
        function obj = registerNewOperationalPlan(obj, operationalPlan)
            %Anadimos el plan de vuelo al registro
            obj.OperationalPlans(end+1) = operationalPlan;
        end
        

        %Conexion con el master de ROS
        function ConnectWithROSMaster(obj)
            try
                rosinit(obj.rosMasterIp, obj.rosMasterPort);
            catch
                disp("ROS ya está iniciado. Reiniciando la conexión ...");
                rosshutdown;
                rosinit(obj.rosMasterIp, obj.rosMasterPort);
            end

            %Creacion del suscriptor para insertar los drones
            [obj.rosPublisher, obj.rosMessage] = rospublisher("/god/insert");
        end


        function LaunchOperationPlan(obj, droneOperator, operationalPlan)
            initPos = obj.GenerateRandomLocaton();
            operationalPlan.Status = 'Launching...';
            obj.AddDroneModelToGazebo(droneOperator, operationalPlan.DroneVehicle.DroneVehicleId, initPos);
            obj.RunSimulinkModel(operationalPlan, operationalPlan.DroneVehicle.DroneVehicleId);
        end


        %Anadir el SDF de un drone a Gazebo
        function AddDroneModelToGazebo(obj,droneOperator,id,initPos)
            %Definimos que SDF debe usarse y generamos el modelo del drone
            if droneOperator.OperatorName == "usuario"
                droneSDF = sprintf(obj.userDroneModel, id, initPos(1), initPos(2), initPos(3), id, id, id);
            else
                droneSDF = sprintf(obj.droneModel, id, initPos(1), initPos(2), initPos(3), id, id, id);
            end
            %Generamos el mensaje y lo enviamos por el topico
            obj.rosMessage.Data = droneSDF;
            send(obj.rosPublisher, obj.rosMessage);
        end


        %Simulink model
        function RunSimulinkModel(obj, operationalPlan, id)
            simulModelConfig = Simulink.SimulationInput(obj.sim_ModelName);
            simulModelConfig = simulModelConfig.setBlockParameter(obj.sim_pub_bus_command, "Topic", "/drone/"+id+"/bus_command");
            simulModelConfig = simulModelConfig.setBlockParameter(obj.sim_sub_camera, "Topic", "/drone/"+id+"/onboard_camera/image_raw");
            simulModelConfig = simulModelConfig.setBlockParameter(obj.sim_sub_odometry, "Topic", "/drone/"+id+"/odometry");
            %simulModelConfig = simulModelConfig.setPostSimFcn(@(operationalPlan) operationalPlan.UpdateStatusAfterExecution);
            operationalPlan.SimulinkInput = simulModelConfig;
            operationalPlan.BatchsimOutput = batchsim(simulModelConfig, 'ShowProgress','on', 'SetupFcn', @obj.InitRandom);
        end


        %Funcion para generar una posicion aleatoria en el mapa no ocupada
        %por ningun otro drone
        function pos = GenerateRandomLocaton(obj)
            xy = randi([-4, 4], 1, 2);
            while obj.dronesUsedPositions(xy(1)+5,xy(2)+5) == 1
                xy = randi([-4, 4], 1, 2);
            end
            
            obj.dronesUsedPositions(xy(1)+5,xy(2)+5) = 1;   
            pos = [xy(1) xy(2) 1];
        end


        %Funcion para que los worker establezcan su generador de numeros
        %aleatorios
        function InitRandom(obj)
            %Para que cada vez que se genere un worker se levante
            %con un número aleatorio en el nombre
            rng('shuffle');
        end

    end
end

