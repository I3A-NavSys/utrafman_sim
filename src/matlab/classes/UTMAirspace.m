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

        %Flight Planner service
        %S_FlightPlansPlanner FlightPlansPlanner;
        
        %ROS network info
        rosMasterIp string
        rosMasterPort uint32 = 11311;

        %Gazebo Clock
        GClock_sub;                 %Subscriber to Gazebo clock
        GClock_Upt_timer;           %Timer to update Gazebo clock
        Gclock = -1;                     %Gazebo clock value

        saveobj_timer
    end
    
    methods
        %Class constructor
        function obj = UTMAirspace()
            %Load ROS configuration variables
            run(fullfile("./config/ros.m"));
            run(fullfile(""));
            obj.rosMasterIp = ROS_IP;

            %Connect with ROS master
            obj.ConnectWithROSMaster();

            %Create instances of objects
            obj.S_Registry = S_Registry();
            obj.S_Monitoring = S_Monitoring();
            
            %If a pool is active, use parallel evaluation. If not, use
            %shared interpreter
            if ~isempty(gcp('nocreate'))
                obj.gct_Registry = parfeval(gcp,@execute,0,obj.S_Registry,"192.168.1.131");
                obj.gct_Monitoring = parfeval(gcp,@execute,0,obj.S_Monitoring,"192.168.1.131");
            else
                obj.S_Registry.execute(obj.rosMasterIp);
                obj.S_Monitoring.execute(obj.rosMasterIp);
            end

            %obj.S_FlightPlansPlanner = FlightPlansPlanner(obj);

            %obj.saveobj_timer = timer("TimerFcn", @obj.saveUTMobject ,"ExecutionMode", "fixedRate", "Period", 60);
            %start(obj.saveobj_timer);
        end

        %Connection with ROS Master
        function ConnectWithROSMaster(obj)
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
            for i=1:size(obj.S_Registry.drones,2)
                drone = obj.S_Registry.drones(i);
                drone.removeDrone();
            end
        end

        %To sabe UTMAirspace object data
        function obj = saveUTMobject(obj, timer, time)
            state = warning;
            warning('off','all');
            save("simulations-results/utm", 'obj');
            warning(state);
        end
    end
end

