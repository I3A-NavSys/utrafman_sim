%Registry class maintains the state of the airspace and an update registry of the entities in the airspace
classdef S_Registry < handle

    properties
        %Operators in the airspace
        operators = ros.msggen.utrafman_main.Operator.empty;        %Array of Operators objects
        operatorLastId uint32 = 0;      %Last operatorId assigned
        
        %Drones in the airspace
        uavs = ros.msggen.utrafman_main.Drone.empty;              %Array of Drone objects
        uavLastId uint32 = 0;         %Last droneId assigned

        %Flight plans in the airspace
        flightPlans = ros.msggen.utrafman_main.Uplan.empty;         %Array of FlightPlan (ordered queue using DTTO)
        flightPlanLastId uint32 = 0;

        %ros publishers and messages
        node
        ros_registry_serv_operators
        ros_registry_serv_uavs
        ros_registry_serv_fps

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
            obj.ros_registry_serv_operators = ros.ServiceServer(obj.node,"/service/registry/reg_new_operator","utrafman_main/reg_new_operator",@obj.regNewOperator);
            obj.ros_registry_serv_uavs = ros.ServiceServer(obj.node, "/service/registry/reg_new_uav", "utrafman_main/reg_new_uav", @obj.regNewUAV);
            obj.ros_registry_serv_fps = ros.ServiceServer(obj.node,"/service/registry/reg_new_fp","utrafman_main/FlightPlan",@obj.regNewFlightPlan);
            pause(Inf);
        end
        
        %Register a new operator in the registry
        function res = regNewOperator(obj, ss, req, res)
            %Compute new operatorId
            id = obj.operatorLastId + 1;
            obj.operatorLastId = id;
            %Assign operatorId
            req.Id = id;
            %Signup in the registry
            obj.operators(id) = req;
            %Response
            res = req;
        end

        %Register a new drone in the registry
        function obj = regNewUAV(obj, UAV)
            %Commpute new uavId
            id = obj.uavLastId + 1;
            obj.uavLastId = id;
            %Assign uavId
            UAV.droneId = id;
            %Signup in the registry
            obj.uavs(id) = UAV;
            %Generate SDF model to be inserted in Gazebo
            UAV.generateSDF();
            %Add model to Gazebo
            obj.ros_droneInsert_msg.Data = UAV.sdf;
            send(obj.ros_droneInsert_pub, obj.ros_droneInsert_msg);
            %Pause is needed to avoid message queue deletion
            pause(0.2);
            %Subscribe to drone topics
            UAV.subToTelemety();
            UAV.pubsubToFlightPlan();
        end

        %Function to register a new flight plan
        function res = regNewFlightPlan(obj, ss, req, res)
            %Compute new flightPlanId
            id = obj.flightPlanLastId + 1;
            %Assign flightPlanLastId
            obj.flightPlanLastId = id;
            %obj.flightPlans(id) = req.
            %Response
            res.Fp = req.Fp;
            res.Status = 1;
        end
    end
end

