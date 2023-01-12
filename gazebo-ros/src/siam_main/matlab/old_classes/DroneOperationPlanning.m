%Entidad que planifica los vuelos en el espacio aéreo, correspondiente con
%el bloque "Drone Operation Planning"

classdef DroneOperationPlanning < handle
    
    properties
        %Ultimos IDs asignados. Ninguno de los IDs será 0
        lastDroneOperatorId = 0;
        lastOperationalPlanId = 0;
        lastDroneVehicleId = 0;

        %Para conectar con ROS
        rosMasterIp = "192.168.1.131";
        rosMasterPort = 11311;

        %Subs/Pubs y mensjajes de ROS
        rosInserterPub;
        rosInserterMsg;
        rosRemoverPub;
        rosRemoverMsg;

        %Modelos SDF de los drones
        droneModel;
        userDroneModel;
        dronesUsedPositions

        %Zona de spawn de drones
        spawn_nw
        spawn_se
        maxIndex
        accumIndex
        
        %Simulink y bloques de Simulink con los parametros de ROS
        sim_ModelName;
        sim_pub_bus_command;
        sim_sub_camera;
        sim_sub_odometry;

        %SimulinkInputArray para las simulaciones
        SimulationInputArray = Simulink.SimulationInput.empty
        SimulationFuture
    end
    
    methods
        %Constructor de la clase
        function obj = DroneOperationPlanning(simulink_model, max_num_drones, spawn_nw, spawn_se)
            %Zona de spawn de drones
            obj.spawn_nw = spawn_nw;
            obj.spawn_se = spawn_se;
            obj.maxIndex = max([obj.spawn_nw, obj.spawn_se]) + (0 - min([obj.spawn_nw, obj.spawn_se])) + 1;
            obj.accumIndex = (0 - min([obj.spawn_nw, obj.spawn_se]) + 1);
            obj.dronesUsedPositions = zeros(obj.maxIndex, obj.maxIndex);
            

            %Obtencion de los modelos de drones para Gazebo
            obj.droneModel = fileread('../../models/dronechallenge_models/drone/model_template_1_simple.sdf');
            obj.userDroneModel = fileread('../../models/dronechallenge_models/drone/model_template_2.sdf');
            
            %Nombre del modelo de Simulink y de los bloques
            obj.sim_ModelName = simulink_model;

            obj.sim_pub_bus_command = obj.sim_ModelName + "/drone simulator/command bus/pub_bus_command";
            obj.sim_sub_camera = obj.sim_ModelName + "/drone simulator/camera bus/ROS communication/sub_camera";
            obj.sim_sub_odometry = obj.sim_ModelName + "/drone simulator/imu bus/sub_odometry";
            
            open_system(obj.sim_ModelName);

            %Conexion con el master de ROS
            obj.ConnectWithROSMaster();

            %Inicio de la pool de workers, o si ya está iniciada, parada y reinicio
            try
                parpool(max_num_drones);
            catch
                poolActual = gcp();
                if poolActual.NumWorkers == max_num_drones
                    disp("Pool de workers ya en ejecución. Descartando el inicio.");
                else 
                    disp("Pool de workers ya en ejecución con otro distinto número de workers. Reiniciando la pool.");
                    delete(gcp("nocreate"));
                    parpool(max_num_drones);
                end
            end
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
            [obj.rosInserterPub, obj.rosInserterMsg] = rospublisher("/god/insert");
            [obj.rosRemoverPub, obj.rosRemoverMsg] = rospublisher("/god/remove");
        end


        %Registro de un nuevo operador
        function obj = registerNewOperator(obj, newOperator)
            obj.lastDroneOperatorId = obj.lastDroneOperatorId + 1;
            newOperator.DroneOperatorId = obj.lastDroneOperatorId;
            obj.Operators(end+1) = newOperator;
        end


        %Registro de un nuevo vehiculo para un operador de vuelo
        function registerNewVehicle(obj, droneVehicle)
            %Asignamos una matricula al drone
            obj.lastDroneVehicleId = obj.lastDroneVehicleId + 1;
            droneVehicle.DroneVehicleId = obj.lastDroneVehicleId;

            %Generamos su simulacion en Simulink
            obj.CreateSimulinkSimulationInput(droneVehicle, droneVehicle.DroneVehicleId);

            %Generamos su posicion inicial
            initPos = obj.GenerateRandomLocaton();
            %Anadimos a Gazebo el modelo de drone
            obj.AddDroneModelToGazebo(droneVehicle.DroneOperator, droneVehicle.DroneVehicleId, initPos);

        end


        %Registro de un nuevo plan de operacion para un operador
        function obj = registerNewOperationalPlan(obj, operationalPlan)
            %Anadimos el plan de vuelo al registro
%             obj.OperationalPlans(end+1) = operationalPlan;
        end


        function LaunchOperationPlan(obj, droneOperator, operationalPlan)
%             operationalPlan.Status = 'Launching...';
%             obj.RunSimulinkModel(operationalPlan, operationalPlan.DroneVehicle.DroneVehicleId);
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
            obj.rosInserterMsg.Data = droneSDF;
            send(obj.rosInserterPub, obj.rosInserterMsg);
        end


        %Eliminamos en modelo de Gazebo
        function RemoveModelFromGazebo(obj,droneVehicle)
            string = ""+droneVehicle.DroneVehicleId+"";
            %Generamos el mensaje y lo enviamos por el topico
            obj.rosRemoverMsg.Data = string;
            send(obj.rosRemoverPub, obj.rosRemoverMsg);
        end


        %Creacion del objeto SimulationInput para su posterior simulacion
        function CreateSimulinkSimulationInput(obj, droneVehicle, id)
            %Asginamos el modelo a la simulacion
            simulModelConfig = Simulink.SimulationInput(obj.sim_ModelName);
            %Cambiamos los parametros de los bloques de Simulink
            simulModelConfig = simulModelConfig.setBlockParameter(obj.sim_pub_bus_command, "Topic", "/drone/"+id+"/bus_command");
            simulModelConfig = simulModelConfig.setBlockParameter(obj.sim_sub_camera, "Topic", "/drone/"+id+"/onboard_camera/image_raw");
            simulModelConfig = simulModelConfig.setBlockParameter(obj.sim_sub_odometry, "Topic", "/drone/"+id+"/odometry");
            %Asignamos sus datos
            simulModelConfig = simulModelConfig.setVariable('droneVehicle', droneVehicle);
            %Almacenamos los datos en el vehiculo y en el planificador
            droneVehicle.SimulationInput = simulModelConfig;
            obj.SimulationInputArray(id) = simulModelConfig;
            warning('off','shared_robotics:robotutils:common:SavedObjectInvalid');

            %simulModelConfig = simulModelConfig.setPostSimFcn(@(x) test(x));
            %operationalPlan.BatchsimOutput = batchsim(simulModelConfig, 'ShowProgress','on', 'SetupFcn', @obj.InitRandom);

            %Establecimiento de un timer
%             operationalPlan.FinishTimer = timer;
%             operationalPlan.FinishTimer.Period = 10;
%             operationalPlan.FinishTimer.ExecutionMode = 'fixedDelay';
%             operationalPlan.FinishTimer.TimerFcn = @operationalPlan.CheckFinishStatus;
%             start(operationalPlan.FinishTimer)
        end

        
        %Puesta en ejecucion de la simulacion
        function LaunchSimulation(obj)
            obj.SimulationFuture = parsim(obj.SimulationInputArray, 'RunInBackground', 'on', 'ShowSimulationManager','on');
        end


        %Funcion para generar una posicion aleatoria en el mapa no ocupada
        %por ningun otro drone
        function pos = GenerateRandomLocaton(obj)
            x = randi([obj.spawn_nw(1), obj.spawn_se(1)], 1, 1);
            y = randi([obj.spawn_nw(2), obj.spawn_se(2)], 1, 1);
%             x = -5;
%             y = -4;
%             x = -70;
%             y = -20;

            while obj.dronesUsedPositions(x+obj.accumIndex,y+obj.accumIndex) == 1
                x = randi([obj.spawn_nw(1), obj.spawn_se(1)], 1, 1);
                y = randi([obj.spawn_nw(2), obj.spawn_se(2)], 1, 1);
            end
            
            obj.dronesUsedPositions(x+obj.accumIndex,y+obj.accumIndex) = 1;   
            pos = [x y 1];
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

