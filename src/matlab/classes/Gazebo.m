%UTM Airspace class represent and entire airspace with all the entities, services and information needed to manage the airspace
classdef UTMAirspace < handle
    
    properties
        %Entities needed in the airspace

        %Registry service
        S_Registry S_Registry;
        gct_Registry

        %Monitoring service
        S_Monitoring S_Monitoring;
        gct_Monitoring
        
        %ROS network info
        rosMasterIp string
        rosMasterPort uint32 = 11311;

        %Gazebo Clock
        Gclock = -1;                %Gazebo clock value
        GClock_sub;                 %Subscriber to Gazebo clock
        GClock_Upt_timer;           %Timer to update Gazebo clock

        %To automatically finish simulations
        finish_simulation_time = inf;
        finish_simulation_timer;
    end
    
    methods
        %Initialisation of UTM Airspace
        function obj = UTMAirspace()

            %Load ROS configuration variables
            global ROS_MASTER_IP;
            obj.rosMasterIp = ROS_MASTER_IP;

            %Connect with ROS master
            obj.connectWithROSMaster();

            %Create instances of objects
            obj.S_Registry = S_Registry();
            obj.S_Monitoring = S_Monitoring();
            
            %If a pool is active, use parallel evaluation. If not, use shared interpreter
            if ~isempty(gcp('nocreate'))
                obj.gct_Registry = parfeval(gcp,@execute,0,obj.S_Registry,"192.168.1.131");
                obj.gct_Monitoring = parfeval(gcp,@execute,0,obj.S_Monitoring,"192.168.1.131");
            else
                obj.S_Registry.execute(obj.rosMasterIp);
                obj.S_Monitoring.execute(obj.rosMasterIp);
            end
        end

        %Connection with ROS Master
        function connectWithROSMaster(obj)
            try
                rosinit(obj.rosMasterIp, obj.rosMasterPort);
            catch
                disp("ROS is already initialised. Restarting connection...");
                rosshutdown;
                rosinit(obj.rosMasterIp, obj.rosMasterPort);
                disp("ROS main connection established.");
            end

            %Suscribe to Gazebo clock
            obj.GClock_sub = rossubscriber("/clock");

            %Configure timer to update Gazebo clock every 1sec
            %Timer is used because clock topic is published with a very high frequency
            obj.GClock_Upt_timer = timer("TimerFcn", @obj.updateGclock,"ExecutionMode","fixedRate");

            %Start timer
            start(obj.GClock_Upt_timer);

            % %Wait until Gazebo clock has a value
            while(obj.Gclock == -1)
                pause(0.1)
            end
        end
        
        %Update of Gazebo Clock value
        function obj = updateGclock(obj, timer, time)
            [msg, status] = receive(obj.GClock_sub, 0.1);
            %If status is true, the message was received
            if status
                obj.Gclock = msg.Clock_.Sec;
            end
        end

        %To remove all models from simulation
        function obj = removeAllModelsFromSimulation(obj)
            client = rossvcclient('/godservice/remove_model');

            for i = obj.S_Registry.uavs
                drone = i;
                msg = rosmessage(client);
                msg.UavId = drone.Id;
                call(client, msg, 'Timeout', 10);
            end
        end

        %To set the automatically finish simulation time
        %Maintains the greatest time passed on all method calls
        function setFinishSimulationTime(obj, finish_time)
            if (obj.finish_simulation_time == inf) || (finish_time > obj.finish_simulation_time)
                obj.finish_simulation_time = finish_time;
            end
            if isempty(obj.finish_simulation_timer)
                obj.finish_simulation_timer = timer("TimerFcn", @obj.checkIfTimeToFinishSimulation,"ExecutionMode","fixedRate", "Period",5);
                start(obj.finish_simulation_timer);
            end
        end

        %Check if is time to finish the simulation
        function obj = checkIfTimeToFinishSimulation(obj, timer, time)
            if obj.Gclock > obj.finish_simulation_time
                obj.finishSimulation();
            end
        end

        %To finish the simulation
        function finishSimulation(obj)
            stop(obj.finish_simulation_timer);
            stop(obj.GClock_Upt_timer);
            obj.pauseSimulation();
            pause(1);
            rosshutdown;
            disp("Finishing simulation");
            timer; stop(timerfind); delete(timerfind);
            global SP;
            SP = SimulationProcesser(obj);
        end

        function pauseSimulation(obj)
            ros_pause_physics = rossvcclient('/gazebo/pause_physics');
            msg = ros_pause_physics.rosmessage;
            if isServerAvailable(ros_pause_physics)
                a = call(ros_pause_physics,msg,"Timeout",5);
            else
                error("Error pausing Gazebo simulation");
            end
        end
    end
end

