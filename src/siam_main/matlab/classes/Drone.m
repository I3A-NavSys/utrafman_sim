classdef Drone < handle

    properties
        UTM

        droneId
        model
        operator

        flightPlan
        status

        initLoc
        loc
        locs = ros.msggen.siam_main.Telemetry.empty(0);

        droneModel
        sdf

        %ROS
        %timer_Upd_Telemetry
        ros_Telemetry_sub
        %ros_Telemetry_msg

        %timer_Upd_Status
        ros_flightPlans_pub
        %ros_flightPlans_sub
        %ros_flightPlans_msg

        %SimulationInput
    end
    
    methods
        function obj = Drone(utm, model, initLoc)
            obj.UTM = utm;
            obj.model = model;
            obj.initLoc = initLoc;
        end

        %Generate SDF model to be used in Gazebo
        function obj = generateSDF(obj)
            obj.selectModel(); %Select drone model
            obj.sdf = sprintf(obj.droneModel, obj.droneId, obj.initLoc(1), obj.initLoc(2), obj.initLoc(3), obj.droneId, obj.droneId, obj.droneId);
        end

        %Select Gazebo model to insert
        function obj = selectModel(obj)
            switch obj.model
                otherwise
                    obj.droneModel = fileread('../models/dronechallenge_models/drone/model_template_1_simple.sdf');
            end
        end

        %Subcription to the drone odometry topic
        function obj = subToTelemety(obj, timer, time)
            %pause(3);
            %Subscription with callback
            obj.ros_Telemetry_sub = rossubscriber(sprintf('/drone/%d/telemetry', obj.droneId),"siam_main/Telemetry",@obj.updateLoc);
        end

        %Publish to the drone flight plan 
        function obj = pubsubToFlightPlan(obj, timer, time)
            %Publisher to send Uplans
            obj.ros_flightPlans_pub = rospublisher(sprintf('/drone/%d/uplan', obj.droneId),"siam_main/Uplan");
        end

        %Callback to update location of the drone
        function updateLoc(obj, sub, msg)
            %Location update
            obj.loc = [msg.Pose.Position.X msg.Pose.Position.Y msg.Pose.Position.Z];
            %Telemetry message saving
            obj.locs(end+1) = msg;
        end

        %Not in use
        %Callback to update status of the drone
        function obj = updateStatus(obj, timer, time)
            if isempty(obj.flightPlan)
                return;
            end
            fp = obj.flightPlan;
            [obj.ros_flightPlans_msg, status] = receive(obj.ros_flightPlans_sub, 0.2);
            if status
                switch obj.ros_flightPlans_msg.Status
                    case 0
                        obj.status = 0;
                        fp.status = 0;
                    case 1
                        obj.status = 1;
                        fp.status = 1;
                    case 2
                        obj.status = 2;
                        fp.status = 2;
                        obj.flightPlan = [];
                    otherwise
                        obj.status = -1;
                end
            end
        end

        %To remove models from the simulation
        function obj = removeDrone(obj)
            %if obj.status ~= -1
                % First: send a topic message to the Gazebo model
                pub = rospublisher("/drone/"+ obj.droneId + "/kill", "std_msgs/Bool");
                msg = rosmessage(pub);
                send(pub,msg);
                %Second: call Gazebo remove model service
                client = rossvcclient("/gazebo/delete_model");
                msg = rosmessage(client);
                msg.ModelName = "drone_" + obj.droneId;
                call(client,msg,"Timeout",3);
                % Mark drone as removed
                obj.status = -1;
            %end
        end
    end

end

