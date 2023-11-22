% U-Space Registry Service class. 
% Maintains a registry of:
%   - operators
%   - UAVs
%   - flight plans
classdef USpace_registrator < handle

properties

    % Operators in the airspace
    operators = ros.msggen.utrafman.Operator.empty;  % Vector of Operator objects
    operator_last_ID  uint32  = 0;                   % ID assigned to the last operator registered
    
    % UAVs in the airspace
    UAVs = ros.msggen.utrafman.UAV.empty;            % Vector of UAV objects
    UAV_last_ID uint32 = 0;                          % ID assigned to the last UAV registered

    % Flight plans in the airspace
    flight_plans = ros.msggen.utrafman.Uplan.empty;  % Vector of FlightPlan (ordered queue using DTTO)
    flight_plan_last_ID uint32 = 0;                  % ID assigned to the last flight plan registered

        
    % ROS interface
    gz Gazebo                      % handle to Gazebo connector
    ROSnode                        % ROS Node handler
    ROSname                        % ROS Node name
    ROScli_deploy_UAV              % Service Client to insert models in Gazebo
    ROSpub_new_uav_advertise       % New UAV registration advertiser
    ROSsrv_register_operator       % Service to register a new operator
    ROSsrv_register_UAV            % Service to register a new UAV
    ROSsrv_register_FP             % Service to register a new FP
    ROSsrv_get_operators           % Service to get the list of operators
    ROSsrv_get_uavs                % Service to get the list of UAV
    ROSsrv_get_fps                 % Service to get the list of FPs

end

methods


% Class constructor
function obj = USpace_registrator(gz)
    obj.gz = gz;

    % Stopping previous U-Space Registry Services
    obj.ROSname = '/utm/registrators/USpace_registrator';
    ROSnodes = rosnode('list');
    if any(strcmp(ROSnodes, obj.ROSname))
        rosnode('kill', obj.ROSname);
        disp("Previous U-Space Registry Service was stopped.");
    end
    
    % Starting U-Space Registry Service
    obj.start;

end


function obj = start(obj)

    % ROS node
    obj.ROSnode = ros.Node("/utm/registrators/USpace_registrator",obj.gz.ROS_MASTER_IP,11311);

    % Connecting to ROS service client to insert UAV into the simulator
    obj.ROScli_deploy_UAV = ros.ServiceClient(obj.ROSnode,"/utm/airspace/deploy_UAV");

    %Initializate ROS Publisher for new UAV advertise
    obj.ROSpub_new_uav_advertise = ros.Publisher(obj.ROSnode,"/registry/new_uav_advertise","utrafman/UAV");

    %Initialize ROS service servers to register into registry
    obj.ROSsrv_register_operator = ros.ServiceServer(obj.ROSnode,"/utm/services/registry/register_operator","utrafman/register_operator",@obj.regNewOperator);
    obj.ROSsrv_register_UAV = ros.ServiceServer(obj.ROSnode, "/utm/services/registry/register_UAV", "utrafman/register_UAV", @obj.regNewUAV);
    obj.ROSsrv_register_FP = ros.ServiceServer(obj.ROSnode,"/utm/services/registry/register_FP","utrafman/register_FP",@obj.regNewFlightPlan);
    
    %Initialize ROS service servers to get registry info
    obj.ROSsrv_get_operators = ros.ServiceServer(obj.ROSnode, "/utm/services/registry/get_operators", "utrafman/get_operators", @obj.getOperators);
    obj.ROSsrv_get_uavs = ros.ServiceServer(obj.ROSnode, "/utm/services/registry/get_UAVs", "utrafman/get_UAVs", @obj.getUavs);
    obj.ROSsrv_get_fps = ros.ServiceServer(obj.ROSnode, "/utm/services/registry/get_FPs", "utrafman/get_FPs", @obj.getFps);

    disp("U-Space Registry Service is running.");

end


%Register a new operator in the registry
function res = regNewOperator(obj, ss, req, res)
    %Compute new operatorId
    id = obj.operator_last_ID + 1;
    obj.operator_last_ID = id;
    %Assign operatorId
    req.OperatorInfo.Id = id;
    %Signup in the registry
    obj.operators(id) = req.OperatorInfo;
    %Response
    res = req;
end


%Register a new drone in the registry
function res = regNewUAV(obj, ss, req, res)

    %Compute new uavId
    id = obj.UAV_last_ID + 1;
    obj.UAV_last_ID = id;
    
    %Assign uavId
    req.Uav.Id = id;

    %Signup in the registry
    obj.UAVs(id) = req.Uav;
    
    %Generate SDF model to be inserted in Gazebo
    sdf_model = obj.generateSDF(req.Uav, req.InitPos);

    %Add model to Gazebo
    msg = obj.ROScli_deploy_UAV.rosmessage;
    msg.ModelSDF = sdf_model;

    %Check if Gazebo god service is available and make the request
    if isServerAvailable(obj.ROScli_deploy_UAV)
        res = call(obj.ROScli_deploy_UAV,msg,"Timeout",3);
    else
        error("Service 'insert_model' not available on network");
    end

    %Advertise about new UAV added to the registry
    send(obj.ROSpub_new_uav_advertise,req.Uav);

    %Send response
    res = req;    

end


%Function to register a new flight plan
function res = regNewFlightPlan(obj, ss, req, res)
    %Compute new flight_plan_lastid
    id = obj.flight_plan_last_ID + 1;
    %Assign flight_plan_lastid
    obj.flight_plan_last_ID = id;
    req.Fp.FlightPlanId = id;
    %Save in the registry
    obj.flight_plans(id) = req.Fp;
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
        res.Uavs = obj.UAVs;
    else
        %Check if the UAV id exists
        if length(obj.UAVs) >= req.UavId
            res.Uavs = obj.UAVs(req.UavId);
        end
    end
end


%Function to get a or the list of flightplans in the registry
function res = getFps(obj, ss, req, res)
    %Check if fp ID is 0 (to get the entire list)
    if (req.FpId == 0)
        res.Fps = obj.flight_plans;
    else
        %Check if the FP id exists
        if length(obj.flight_plans) >= req.FpId
            res.Fps = obj.flight_plans(req.FpId);
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
            global UTRAFMAN_DIR;
            file = fileread(fullfile(UTRAFMAN_DIR,...
                '/gazebo-ros/src/utrafman/models/DC/abejorro/model_simplified_template.sdf'));
    end
end


end %methods
end %classdef