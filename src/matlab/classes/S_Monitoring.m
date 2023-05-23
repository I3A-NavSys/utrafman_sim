%Monitoring Service class

classdef S_Monitoring< handle
    properties
        %UAV in the airspace
        uavs = struct([]);                              %Array of UAV (struct format)
        uavs_telemetry_subs = ros.Subscriber.empty;     %Array of ROS subscribers for each UAV to receive telemetry data

        %ROS structs
        node                                            %Node
        ros_subs_new_uavs                               %Subscrition to new UAV advertiser topic to know when a new UAV is added
        ros_srv_get_telemetry                           %Service server to get the Telemetry data of a UAV
        ros_srv_get_currentloc                          %Service server to get the current Telemetry data of a UAV
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
            %Initializate ROS Service server to get telemetry of a UAV
            obj.ros_srv_get_telemetry = ros.ServiceServer(obj.node, "/service/monitoring/get_telemetry", "utrafman_main/mon_get_locs", @obj.getLocs);
            %Initializate ROS Service server to get the current location of a UAV
            obj.ros_srv_get_currentloc = ros.ServiceServer(obj.node, "/service/monitoring/get_current_loc", "utrafman_main/mon_get_locs", @obj.getCurrentLoc);

            disp("Monitoring service has been initialized");
            job = getCurrentJob;
            %Check if is running in a worker and make it infinite
            if ~isempty(job)
                pause(Inf);
            end
        end

        %This function is called when new UAV is registered
        function newUav(obj, sub, msg)
            %Get ID
            id = msg.Id;
            %UAV data
            uav.reg_msg = msg;                                             %UAV ROS msg
            uav.loc = ros.msggen.utrafman_main.Telemetry;                  %To store the last location sent by the UAV
            uav.telemetry = ros.msggen.utrafman_main.Telemetry.empty;
            %Save UAV data
            if (id == 1)
                obj.uavs = uav;
            else
                obj.uavs = [obj.uavs uav];
            end
            %UAV Telemetry data subscription
            obj.uavs_telemetry_subs(id) = ros.Subscriber(obj.node, "/drone/"+id+"/telemetry", "utrafman_main/Telemetry", {@obj.newTelemetryData, id});
        end

        %This function is called every time a UAV sends telemetry data
        function newTelemetryData(obj, sub, msg, id)
            %Store last location and add telemetry to the list of messages
            obj.uavs(id).loc = msg;
            obj.uavs(id).telemetry(end+1) = msg;
        end

        %Function to get locs of a UAV
        function res = getLocs(obj, ss, req, res)
            %Check if operator ID is different from 0 (a UAVId must be defined)
            if (req.UavId ~= 0)
                %Check if UAVId exists
                if length(obj.uavs) >= req.UavId
                    res.Telemetry = obj.uavs(req.UavId).telemetry;
                end
            end
        end

        %Function to get current loc of a UAV
        function res = getCurrentLoc(obj, ss, req, res)
            %Check if operator ID is different from 0 (a UAVId must be defined)
            if (req.UavId ~= 0)
                %Check if UAVId exists
                if length(obj.uavs) >= req.UavId
                    res.Telemetry = obj.uavs(req.UavId).loc;
                end
            end
        end
    end
end

