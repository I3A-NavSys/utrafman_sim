% Operator classes represent operators in the context of U-space.
% Each operator has a drone garage where it stores its UAVs.

classdef SimpleOperator < handle

properties
    name         string            % Operator name
    id           uint32            % Unique ID (provided by the U-space registry service)
    drone_garage UAVProperties     % Array of UAV objects references

    % ROS interface
    gz Gazebo                      % handle to Gazebo connector
    ROSnode                        % ROS Node
    ROScli_reg_operator            % Service client to register itself as operators
    ROScli_reg_UAV                 % Service client to register a new UAVs
    ROScli_reg_FP                  % Service client to register a new FP

end

methods


%Class constructor
function obj = SimpleOperator(gz,operator_name)

    obj.gz = gz;
    obj.name = operator_name;
    
    % ROS node
    obj.ROSnode = ros.Node("/utm/operators/"+obj.name,gz.ROS_MASTER_IP,11311);

    % ROS service client to register itself as operator
    obj.ROScli_reg_operator = ros.ServiceClient(obj.ROSnode,"/utm/services/registry/register_operator");

    % ROS service client to register UAVs
    obj.ROScli_reg_UAV = ros.ServiceClient(obj.ROSnode,"/utm/services/registry/register_UAV");

    % ROS service client to register flight plans
    obj.ROScli_reg_FP = ros.ServiceClient(obj.ROSnode,"/utm/services/registry/register_FP");
    
    %Sign up in the registry
    if isServerAvailable(obj.ROScli_reg_operator)
        req = obj.ROScli_reg_operator.rosmessage;
        req.OperatorInfo.Name = obj.name;
        %Call service
        res = call(obj.ROScli_reg_operator, req, "Timeout", 3);
    end
    %Get operator ID
    obj.id = res.OperatorInfo.Id;
end


%Register a new drone to the operator adding it to the drone garage
function UAVprop = regNewDrone(obj,model,init_pos)

    %Create UAVProperties
    UAVprop = UAVProperties(obj.id,model,init_pos);

    if isServerAvailable(obj.ROScli_reg_UAV)
        %Create request and fullfillment
        req = obj.ROScli_reg_UAV.rosmessage;
        req.Uav.OperatorId = obj.id;
        req.Uav.Model = model;
        req.InitPos = init_pos;
        %Call service
        res = call(obj.ROScli_reg_UAV, req, "Timeout", 3);
    end
    %Store properties
    UAVprop.UAV_reg_message = res.Uav;
    UAVprop.drone_id = res.Uav.Id;
    UAVprop.pubsubToFlightPlan();
    %Save in the garage
    obj.drone_garage(end+1) = UAVprop;
end


function obj = regNewFP(obj,fp)
    if isServerAvailable(obj.ROScli_reg_FP)
        %Create request
        reg_fp_req = obj.ROScli_reg_FP.rosmessage;
        reg_fp_req.Fp = fp.parseToROSMessage();
        %Call
        reg_fp_res = call(obj.ROScli_reg_FP,reg_fp_req, "Timeout", 3);
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
    

end % methods 
end % classdef

