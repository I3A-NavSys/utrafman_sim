%Class for a UAV in the simulation

classdef UAVProperties < handle

    properties
        drone_id uint32             % Unique ID (provided by the U-space registry service)
        model    string                
        operator_id  uint32         % Operator ID of the drone
        UAV_reg_message = ros.msggen.utrafman.UAV;

        status int8                 % Status flag

        init_loc                    % Vector3<double>: Spawn location of the drone in the world 

        %ROS
        ros_telemetry_sub           % ROS Subscriber object reference
        ros_fp_pub                  % ROS Publisher object reference 
    end
    
    methods
        %Constructor of the class
        function obj = UAVProperties(operator_id, model, init_loc)
            obj.operator_id = operator_id;
            obj.model = model;
            obj.init_loc = init_loc;
            obj.status = 0;
        end

        %Publisher for drone flight plans 
        function obj = pubsubToFlightPlan(obj, timer, time)
            obj.ros_fp_pub = rospublisher(sprintf('/drone/%d/uplan', obj.drone_id),"utrafman/Uplan");
        end
    end

end

