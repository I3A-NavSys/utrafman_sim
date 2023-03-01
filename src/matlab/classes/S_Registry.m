%Registry class maintains the state of the airspace and an update registry of the entities in the airspace
classdef S_Registry < handle

    properties
        %Operators in the airspace
        operators = ros.msggen.utrafman_main.Operator.empty;        %Array of Operators objects
        operatorLastId uint32 = 0;                                  %Last operatorId assigned
        
        %UAVs in the airspace
        uavs = ros.msggen.utrafman_main.UAV.empty;                  %Array of Drone objects
        uavLastId uint32 = 0;                                       %Last droneId assigned

        %Flight plans in the airspace
        flightPlans = ros.msggen.utrafman_main.Uplan.empty;         %Array of FlightPlan (ordered queue using DTTO)
        flightPlanLastId uint32 = 0;                                %Last flightPlanId assigned

        %ros structs
        node                            %Node
        ros_model_pub                   %Service Client to insert models in Gazebo
        ros_pub_new_uav_advertise       %New UAV registration advertiser
        ros_srv_reg_operator            %Service to register a new operator
        ros_srv_reg_uav                 %Service to register a new UAV
        ros_srv_reg_fp                  %Service to register a new FP
        ros_srv_get_operators           %Service to get a or the list of operators
        ros_srv_get_uavs                %Service to get a or the list of UAV
        ros_srv_get_fps                 %Service to get a or the list of FPs

    end
    
    methods
        %Class constructor
        function obj = S_Registry()
            disp("Registry service instance created");
        end

        function obj = execute(obj, ROS_MASTER_IP)            
            %Initializate ROS node
            obj.node = ros.Node("registry_service", ROS_MASTER_IP, 11311);

            %Initializate ROS Service Client to insert UAV in the simulator
            obj.ros_model_pub = ros.ServiceClient(obj.node,"/godservice/insert_model");

            %Initializate ROS Published for new UAV advertise
            obj.ros_pub_new_uav_advertise = ros.Publisher(obj.node,"/registry/new_uav_advertise","utrafman_main/UAV");

            %Initialize ROS service servesrs to register into registry
            obj.ros_srv_reg_operator = ros.ServiceServer(obj.node,"/service/registry/reg_new_operator","utrafman_main/reg_new_operator",@obj.regNewOperator);
            obj.ros_srv_reg_uav = ros.ServiceServer(obj.node, "/service/registry/reg_new_uav", "utrafman_main/reg_new_uav", @obj.regNewUAV);
            obj.ros_srv_reg_fp = ros.ServiceServer(obj.node,"/service/registry/reg_new_fp","utrafman_main/reg_new_flightplan",@obj.regNewFlightPlan);
            
            %Initialize ROS service servesrs to get registry info
            obj.ros_srv_get_operators = ros.ServiceServer(obj.node, "/service/registry/get_operators", "utrafman_main/reg_get_operators", @obj.getOperators);
            obj.ros_srv_get_uavs = ros.ServiceServer(obj.node, "/service/registry/get_uavs", "utrafman_main/reg_get_uavs", @obj.getUavs);
            obj.ros_srv_get_fps = ros.ServiceServer(obj.node, "/service/registry/get_fps", "utrafman_main/reg_get_fps", @obj.getFps);

            disp("Registry service has been initialized");
            job = getCurrentJob;
            %Check if is running in a worker and make it infinite
            if ~isempty(job)
                pause(Inf);
            end
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
            sdf_model = obj.generateSDF(req.Uav, req.InitPos);
            %Add model to Gazebo
            insert_msg = obj.ros_model_pub.rosmessage;
            insert_msg.ModelSDF = sdf_model;
            %Check if Gazebo god service is available and make the request
            if isServerAvailable(obj.ros_model_pub)
                insert_res = call(obj.ros_model_pub,insert_msg,"Timeout",3);
            else
                error("Service 'insert_model' not available on network");
            end
            %Advertise about new UAV added to the registry
            send(obj.ros_pub_new_uav_advertise,req.Uav);
            %Send response
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

        %Function to get a or the list of operators in the registry
        function res = getOperators(obj, ss, req, res)
            %Check if operator ID is 0 (to get the entire list)
            if (req.OperatorId == 0)
                res.Operators = obj.operators;
            else
                %Check if the operator id exists
                if length(obj.operators) >= req.OperatorId
                    res.Operators = obj.operators(req.OperatorId);
                end
            end
        end

        %Function to get a or the list of UAVs in the registry
        function res = getUavs(obj, ss, req, res)
            %Check if UAV ID is 0 (to get the entire list)
            if (req.UavId == 0)
                res.Uavs = obj.uavs;
            else
                %Check if the UAV id exists
                if length(obj.uavs) >= req.UavId
                    res.Uavs = obj.uavs(req.UavId);
                end
            end
        end

        %Function to get a or the list of flightplans in the registry
        function res = getFps(obj, ss, req, res)
            %Check if fp ID is 0 (to get the entire list)
            if (req.FpId == 0)
                res.Fps = obj.flightPlans;
            else
                %Check if the FP id exists
                if length(obj.flightPlans) >= req.FpId
                    res.Fps = obj.flightPlans(req.FpId);
                end
            end
        end

         %Generate SDF model to be used in Gazebo
         function sdf_model = generateSDF(obj, uav_msg, init_pos)
            model_file = obj.selectModel(uav_msg.Model);
            sdf_model = sprintf(model_file, uav_msg.Id, init_pos(1), init_pos(2), init_pos(3), uav_msg.Id, uav_msg.Id, uav_msg.Id);
        end

        %Select Gazebo model to insert
        %This will used in the future to add new drone models 
        function file = selectModel(~,model)
            switch model
                otherwise
                    file = fileread('../gazebo-ros/src/utrafman_main/models/drone2/template_model_2_simple.sdf');
            end
        end 

    end
end