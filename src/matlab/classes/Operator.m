%Operator class represents an operator of the drone system. Could be a company or a person. Each operator has a unique ID and a name. Each operator has a drone garage where it stores the drones it owns.

classdef Operator < handle
    properties
        operator_id uint32                %Unique operator ID
        operator_name string              %Operator name
        drone_garage UAVProperties %Array of drone objects references

        ros_node
        ros_reg_operator
        ros_reg_uav
        ros_reg_fp
    end
    
    methods
        %Class constructor
        function obj = Operator(operator_name, ROS_MASTER_IP)
            obj.operator_name = operator_name;
            %Create ROS object
            obj.ros_node = ros.Node("operator_"+operator_name, ROS_MASTER_IP, 11311);
            obj.ros_reg_operator = ros.ServiceClient(obj.ros_node,"/service/registry/reg_new_operator");
            obj.ros_reg_uav = ros.ServiceClient(obj.ros_node,"/service/registry/reg_new_uav");
            obj.ros_reg_fp = ros.ServiceClient(obj.ros_node,"/service/registry/reg_new_fp");
            
            %Sign up in the registry
            if isServerAvailable(obj.ros_reg_operator)
                reg_op_req = obj.ros_reg_operator.rosmessage;
                reg_op_req.OperatorInfo.Name = obj.operator_name;
                %Call service
                reg_op_res = call(obj.ros_reg_operator, reg_op_req, "Timeout", 3);
            end
            %Get operator ID
            obj.operator_id = reg_op_res.OperatorInfo.Id;
        end

        %Register a new drone to the operator adding it to the drone garage
        function uav_prop = regNewDrone(obj, model, init_pos)
            %Create UAVProperties
            uav_prop = UAVProperties(obj, model, init_pos);
            %Check if server is available
            if isServerAvailable(obj.ros_reg_uav)
                %Create request and fullfillment
                reg_uav_req = obj.ros_reg_uav.rosmessage;
                reg_uav_req.Uav.OperatorId = obj.operator_id;
                reg_uav_req.Uav.Model = model;
                reg_uav_req.InitPos = init_pos;
                %Call service
                reg_uav_res = call(obj.ros_reg_uav, reg_uav_req, "Timeout", 3);
            end
            %Store properties
            uav_prop.UAV_reg_message = reg_uav_res.Uav;
            uav_prop.drone_id = reg_uav_res.Uav.Id;
            uav_prop.pubsubToFlightPlan();
            %Save in the garage
            obj.drone_garage(end+1) = uav_prop;
        end

        function obj = regNewFP(obj,fp)
            if isServerAvailable(obj.ros_reg_fp)
                %Create request
                reg_fp_req = obj.ros_reg_fp.rosmessage;
                reg_fp_req.Fp = fp.parseToROSMessage();
                %Call
                reg_fp_res = call(obj.ros_reg_fp,reg_fp_req, "Timeout", 3);
                %Save info
                if (reg_fp_res.Status)
                    fp.flightplan_id = reg_fp_res.Fp.FlightPlanId;
                else
                    fprintf("Flight Plan %d NOT accepted", reg_fp_res.Fp.flightPlanId);
                end
            end
        end

        %Send a flight plan to a UAV
        function sendFlightPlan(obj, fp)
            if isempty(fp.flightplan_id)
                fprintf("Trying to send a FP to UAV without flightplan_id");
                return;
            end
            %Get UAV and send FP
            uav = fp.uav;
            send(uav.ros_fp_pub, fp.parseToROSMessage());
            fp.sent = 1;
        end
        
    end
end

