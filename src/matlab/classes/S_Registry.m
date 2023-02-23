%Registry class maintains the state of the airspace and an update registry of the entities in the airspace
classdef S_Registry < handle

    properties
        %Operators in the airspace
        operators = ros.msggen.utrafman_main.Operator.empty;        %Array of Operators objects
        operatorLastId uint32 = 0;      %Last operatorId assigned
        
        %Drones in the airspace
        uavs = ros.msggen.utrafman_main.UAV.empty;              %Array of Drone objects
        uavLastId uint32 = 0;         %Last droneId assigned

        %Flight plans in the airspace
        flightPlans = ros.msggen.utrafman_main.Uplan.empty;         %Array of FlightPlan (ordered queue using DTTO)
        flightPlanLastId uint32 = 0;

        %ros publishers and messages
        node
        ros_model_pub
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
            %This is executed in a MATLAB worker
            %gct = parfeval(bgp,@jesus.execute,0,"192.168.1.131")
            
            %Initializate ROS node
            obj.node = ros.Node("registry_service", ROS_MASTER_IP, 11311);
            %Initializate ROS Publisher
            obj.ros_model_pub = ros.ServiceClient(obj.node,"/godservice/insert_model");
            %Initialize ROS publishers and messages for airspace's god
            obj.ros_registry_serv_operators = ros.ServiceServer(obj.node,"/service/registry/reg_new_operator","utrafman_main/reg_new_operator",@obj.regNewOperator);
            obj.ros_registry_serv_uavs = ros.ServiceServer(obj.node, "/service/registry/reg_new_uav", "utrafman_main/reg_new_uav", @obj.regNewUAV);
            obj.ros_registry_serv_fps = ros.ServiceServer(obj.node,"/service/registry/reg_new_fp","utrafman_main/reg_new_flightplan",@obj.regNewFlightPlan);
            pause(Inf);
        end
        
        %Register a new operator in the registry
        function res = regNewOperator(obj, ss, req, res)
            %Compute new operatorId
            id = obj.operatorLastId + 1;
            obj.operatorLastId = id;
            %Assign operatorId
            req.OperatorInfo.Id = id;
            %Signup in the registry
            obj.operators(id) = req.OperatorInfo;
            %Response
            res = req;
        end

        %Register a new drone in the registry
        function res = regNewUAV(obj, ss, req, res)
            %Commpute new uavId
            id = obj.uavLastId + 1;
            obj.uavLastId = id;
            %Assign uavId
            req.Uav.Id = id;
            %Signup in the registry
            obj.uavs(id) = req.Uav;
            %Generate SDF model to be inserted in Gazebo
            sdf_model = obj.generateSDF(req.Uav);
            %Add model to Gazebo
            insert_msg = obj.ros_model_pub.rosmessage;
            insert_msg.ModelSDF = sdf_model;
            %Check if Service is available and request
            if isServerAvailable(obj.ros_model_pub)
                insert_res = call(obj.ros_model_pub,insert_msg,"Timeout",3);
            else
                error("Service 'insert_model' not available on network");
            end
            %Send request
            res = req;
        end

        %Function to register a new flight plan
        function res = regNewFlightPlan(obj, ss, req, res)
            %Compute new flightPlanId
            id = obj.flightPlanLastId + 1;
            %Assign flightPlanLastId
            obj.flightPlanLastId = id;
            req.Fp.FlightPlanId = id;
            %Save in the registry
            obj.flightPlans(id) = req.Fp;
            %Response
            res.Fp = req.Fp;
            res.Status = 1;
        end

         %Generate SDF model to be used in Gazebo
        function sdf_model = generateSDF(obj, uav_msg)
            model_file = obj.selectModel(uav_msg.Model);
            sdf_model = sprintf(model_file, uav_msg.Id, 1, 1, 1, uav_msg.Id, uav_msg.Id, uav_msg.Id);
        end

        %Select Gazebo model to insert
        %This will in the future add new drone models 
        function file = selectModel(~,model)
            switch model
                otherwise
                    file = fileread('../gazebo-ros/src/utrafman_main/models/drone2/template_model_2_simple.sdf');
            end
        end 

    end
end

