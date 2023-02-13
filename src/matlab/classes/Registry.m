%Registry class maintains the state of the airspace and an update registry of the entities in the airspace
classdef Registry < handle

    properties
        %Operators in the airspace
        operators = Operator.empty;     %Array of Operators objects
        operatorLastId uint32 = 0;      %Last operatorId assigned
        
        %Drones in the airspace
        drones = Drone.empty;           %Array of Drone objects
        droneLastId uint32 = 0;         %Last droneId assigned

        %Flight plans in the airspace
        flightPlans = ros.msggen.utrafman_main.Uplan.empty;     %Array of FlightPlan (ordered queue using DTTO)
        flightPlanLastId uint32 = 0;                        %Last flightPlanId assigned 

        %ros publishers and messages for airspace's god
        %god is the entity that adds or removes drones from Gazebo world
        ros_droneInsert_pub             %ROS publiser object reference to insert drones in the world
        ros_droneInsert_msg             %ROS message
    end
    
    methods
        %Class constructor
        function obj = Registry()
            %Initialize ROS publishers and messages for airspace's god
             obj.ros_droneInsert_pub = rospublisher('/god/insert','std_msgs/String');
             obj.ros_droneInsert_msg = rosmessage('std_msgs/String');
        end
        
        %Register a new operator in the registry
        function obj = regNewOperator(obj,operator)
            %Compute new operatorId
            id = obj.operatorLastId + 1;
            obj.operatorLastId = id;
            %Assign operatorId
            operator.operatorId = id;
            %Signup in the registry
            obj.operators(id) = operator;
        end

        %Register a new drone in the registry
        function obj = regNewDrone(obj, drone)
            %Commpute new droneId
            id = obj.droneLastId + 1;
            obj.droneLastId = id;
            %Assign droneId
            drone.droneId = id;
            %Signup in the registry
            obj.drones(id) = drone;
            %Generate SDF model to be inserted in Gazebo
            drone.generateSDF();
            %Add model to Gazebo
            obj.ros_droneInsert_msg.Data = drone.sdf;
            send(obj.ros_droneInsert_pub, obj.ros_droneInsert_msg);
            %Pause is needed to avoid message queue deletion
            pause(0.2);
            %Subscribe to drone topics
            drone.subToTelemety();
            drone.pubsubToFlightPlan();
        end

        %Function to register a new flight plan
        function obj = regNewFlightPlan(obj, fp)
            %Compute new flightPlanId
            id = obj.flightPlanLastId + 1;
            obj.flightPlanLastId = id;
            %Assign flightPlanLastId
            fp.flightPlanId = id;
            %Signup in the registry
            obj.InsertFlightPlanQueue(fp)
        end
        
        %Inserts a flight plan in the queue using dtto as order criteria
        function InsertFlightPlanQueue(obj, fp)
            dtto = fp.dtto;
            %Number of FPs in the queue
            qlen = size(obj.flightPlans,2);
            i = 1;
            %Find the position in the queue
            while  i <= qlen && dtto >= obj.flightPlans(i).dtto
                i = i+1;
            end
            %Insert the FP in the queue
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

