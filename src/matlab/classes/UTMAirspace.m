%UTM Airspace class represent and entire airspace with all the entities, services and information needed to manage the airspace
classdef UTMAirspace < handle
    
    properties
        %Entities needed in the airspace

        %Registry service
        S_Registry S_Registry;
        gct_Registry

        %Flight Planner service
        %S_FlightPlansPlanner FlightPlansPlanner;
        
        %ROS network info
        rosMasterIp string
        rosMasterPort uint32 = 11311;

        %Gazebo Clock
        GClock_sub;                 %Subscriber to Gazebo clock
        GClock_Upt_timer;           %Timer to update Gazebo clock
        Gclock = -1;                     %Gazebo clock value
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
            if ~isempty(gcp('nocreate'))
                obj.gct_Registry = parfeval(gcp,@obj.S_Registry.execute,0,"192.168.1.131");
            else
                obj.S_Registry.execute(obj.rosMasterIp);
            end
            %obj.S_FlightPlansPlanner = FlightPlansPlanner(obj);
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
            for i=1:size(obj.S_Registry.drones,2);
                drone = obj.S_Registry.drones(i);
                drone.removeDrone();
            end
        end
    end
end

