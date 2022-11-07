classdef Registry < handle

    properties
        %Properties for operators, drones and flightplans.
        operators = Operator.empty;
        operatorLastId = 0;
        
        drones = Drone.empty;
        droneLastId = 0;

        flightPlans = ros.msggen.siam_main.Uplan.empty; %Ordered queue using DTTO
        %Next_flightPlans = FlightPlan.empty; %Ordered queue using DTTO
        %Waiting_flightPlans = FlightPlan.empty; %Ordered queue using DTTO
        %Finished_flightPlans = FlightPlan.empty; %Ordered queue using DTTO
        flightPlanLastId = 0; %ID for FP

        %To sign up drones
        ros_droneInsert_pub
        ros_droneInsert_msg
    end
    
    methods

        function obj = Registry()
             obj.ros_droneInsert_pub = rospublisher('/god/insert','std_msgs/String');
             obj.ros_droneInsert_msg = rosmessage('std_msgs/String');
        end
        
        function obj = regNewOperator(obj,operator)
            %Compute operatorId
            id = obj.operatorLastId + 1;
            obj.operatorLastId = id;
            %Assign operatorId
            operator.operatorId = id;
            %Signup in the registry
            obj.operators(id) = operator;
        end

        function obj = regNewDrone(obj, drone)
            %Commpute droneId
            id = obj.droneLastId + 1;
            obj.droneLastId = id;
            %Assign droneId
            drone.droneId = id;
            %Signup in the registry
            obj.drones(id) = drone;

            %Generate SDF model
            drone.generateSDF();
            %Add drone to Gazebo
            obj.ros_droneInsert_msg.Data = drone.sdf;
            send(obj.ros_droneInsert_pub, obj.ros_droneInsert_msg);

            %Init drone location updates
            drone.subToTelemety();
            %Init drone flightPlanUpdates
            drone.pubsubToFlightPlan();
        end

        function obj = regNewFlightPlan(obj, fp)
            %Compute flightPlanId
            id = obj.flightPlanLastId + 1;
            obj.flightPlanLastId = id;
            %Assign flightPlanLastId
            fp.flightPlanId = id;

            %Signup in the registry
            obj.InsertFlightPlanQueue(fp)
        end
        
        %Add flightPlan to a queue following dtto
        function InsertFlightPlanQueue(obj, fp)
            dtto = fp.dtto;
            qlen = size(obj.flightPlans,2);
            i = 1;
            while  i <= qlen && dtto >= obj.flightPlans(i).dtto
                i = i+1;
            end
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

