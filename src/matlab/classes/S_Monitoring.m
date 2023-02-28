%Monitoring class receives data from UAV 
classdef S_Monitoring< handle

    properties
        %Drones in the airspace
        uavs;              %Array of UAV objects
        uavs_telemetry_subs = ros.Subscriber.empty;

        %ros publishers and messages
        node
        ros_subs_new_uavs
        ros_srv_get_locs
    end
    
    methods
        %Class constructor
        function obj = S_Monitoring()
            disp("Monitoring service instance created");
        end

        function obj = execute(obj, ROS_MASTER_IP)
            %Initializate ROS node
            obj.node = ros.Node("monitoring_service", ROS_MASTER_IP, 11311);
            %Initializate ROS new UAV subscriber
            obj.ros_subs_new_uavs = ros.Subscriber(obj.node,"/registry/new_uav_advertise", "utrafman_main/UAV", @obj.newUav);

%             %Initialize ROS publishers and messages for airspace's god
%             obj.ros_srv_reg_operator = ros.ServiceServer(obj.node,"/service/registry/reg_new_operator","utrafman_main/reg_new_operator",@obj.regNewOperator); 

             obj.ros_srv_get_locs = ros.ServiceServer(obj.node, "/service/monitoring/get_locs", "utrafman_main/mon_get_locs", @obj.getLocs);


            disp("Registry service has been initialized");
            job = getCurrentJob;
            %Check if is running in a worker
            if ~isempty(job)
                pause(Inf);
            end
        end

        %This function is called when new UAV is registered
        function obj = newUav(obj, sub, msg)
            id = msg.Id;
            obj.uavs(id).reg_msg = msg;
            obj.uavs(id).loc = ros.msggen.utrafman_main.Telemetry;
            obj.uavs(id).locs = ros.msggen.utrafman_main.Telemetry.empty;
            obj.ros_subs_new_uavs(id) = ros.Subscriber(obj.node, "/drone/"+id+"/telemetry", "utrafman_main/Telemetry", {@obj.newTelemetryData, id});
        end

        %This function is called every time a UAV sends telemetry data
        function newTelemetryData(obj, sub, msg, id)
            obj.uavs(id).loc = msg;
             obj.uavs(id).locs(end+1) = msg;
            %disp("Mensaje de telemetria recibido de " + id);
        end

        %Function to get locs of a UAV
        function res = getLocs(obj, ss, req, res)
            %Check if operator ID is different
            if (req.UavId ~= 0)
                if length(obj.uavs) >= req.UavId
                    res.Telemetry = obj.uavs(req.UavId).locs;
                end
            end
        end
    end
end

