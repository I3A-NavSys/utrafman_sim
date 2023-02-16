%Registry class maintains the state of the airspace and an update registry of the entities in the airspace
classdef S_Registry < handle

    properties
        %Operators in the airspace
        operators = Operator.empty;     %Array of Operators objects
        operatorLastId uint32 = 0;      %Last operatorId assigned
        
        %Drones in the airspace
        drones = Drone.empty;           %Array of Drone objects
        droneLastId uint32 = 0;         %Last droneId assigned

        %Flight plans in the airspace
        flightPlans = ros.msggen.siam_main.Uplan.empty;     %Array of FlightPlan (ordered queue using DTTO)
        flightPlanLastId uint32 = 0;                        %Last flightPlanId assigned 

        %ros publishers and messages
        node

        ros_registry_serv_operators
        %ros_registry_pub_operators

        ros_registry_serv_uavs
        %ros_registry_pub_uavs

        ros_registry_serv_fps
        %ros_registry_pub_fps
    end
    
    methods
        %Class constructor
        function obj = Registry()
            disp("Registry service instance created");
        end

        function obj = execute(obj, ROS_MASTER_IP)
            %Initializate ROS node
            obj.node = ros.Node("registry_service", ROS_MASTER_IP, 11311);
            %Initialize ROS publishers and messages for airspace's god
            obj.ros_registry_serv_fps = ros.ServiceServer(obj.node,"/service/registry/reg_new_fp","utrafman_main/FlightPlan",@obj.regNewFlightPlan);
            pause(Inf);
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
        function res = regNewFlightPlan(obj, ss, req, res)
            %Compute new flightPlanId
            id = obj.flightPlanLastId + 1;
            obj.flightPlanLastId = id;
            %Assign flightPlanLastId
            res.Fp = req.Fp;
            res.Status = 1;
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

