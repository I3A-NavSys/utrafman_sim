classdef Registry < handle

    properties
        %Properties for operators, drones and flightplans.
        operators = Operator.empty;     %Array of Operators objects
        operatorLastId uint32 = 0;
        
        drones = Drone.empty;           %Array of Drone objects
        droneLastId uint32 = 0;

        flightPlans = ros.msggen.siam_main.Uplan.empty;     %Array of FlightPlan (ordered queue using DTTO)
        
        flightPlanLastId uint32 = 0;

        ros_droneInsert_pub             %ROS publiser object reference to insert drones in the world
        ros_droneInsert_msg             %ROS message
    end
    
    methods
        %Class constructor
        function obj = Registry()
             obj.ros_droneInsert_pub = rospublisher('/god/insert','std_msgs/String');
             obj.ros_droneInsert_msg = rosmessage('std_msgs/String');
        end
        
        %Function to register a new operator
        function obj = regNewOperator(obj,operator)
            %Compute operatorId
            id = obj.operatorLastId + 1;
            obj.operatorLastId = id;
            %Assign operatorId
            operator.operatorId = id;
            %Signup in the registry
            obj.operators(id) = operator;
        end

        %Function to register a new drone
        function obj = regNewDrone(obj, drone)
            %Commpute droneId
            id = obj.droneLastId + 1;
            obj.droneLastId = id;
            %Assign droneId
            drone.droneId = id;
            %Signup in the registry
            obj.drones(id) = drone;

            %Generate SDF model
            drone.generateSDF();
            %Add drone to Gazebo
            obj.ros_droneInsert_msg.Data = drone.sdf;
            send(obj.ros_droneInsert_pub, obj.ros_droneInsert_msg);
            pause(0.3);
            %Greate a timer to subscribe
            %t = timer('Period',2,'TasksToExecute',1,'TimerFcn', {@drone.subToTelemety, @drone.pubsubToFlightPlan});
            %start(t);
        end

        %Function to register a new flight plan
        function obj = regNewFlightPlan(obj, fp)
            %Compute flightPlanId
            id = obj.flightPlanLastId + 1;
            obj.flightPlanLastId = id;
            %Assign flightPlanLastId
            fp.flightPlanId = id;

            %Signup in the registry
            obj.InsertFlightPlanQueue(fp)
        end
        
        %Add flightPlan to a order queue using dtto
        function InsertFlightPlanQueue(obj, fp)
            dtto = fp.dtto;
            qlen = size(obj.flightPlans,2);
            i = 1;
            while  i <= qlen && dtto >= obj.flightPlans(i).dtto
                i = i+1;
            end
            if i == 1
                obj.flightPlans = [fp obj.flightPlans];
            elseif i == qlen+1
                obj.flightPlans(end+1) = fp;
            else
                obj.flightPlans = [obj.flightPlans(1:i-1) fp obj.flightPlans(i:end)];
            end
        end
        
    end
end

