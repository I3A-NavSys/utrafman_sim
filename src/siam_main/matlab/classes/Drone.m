classdef Drone < handle

    properties
        droneId
        model
        operator

        flightPlan
        status

        initLoc
        loc

        droneModel
        sdf

        %ROS
        timer_Upd_Telemetry
        ros_Telemetry_sub
        ros_Telemetry_msg

        timer_Upd_Status
        ros_flightPlans_pub
        ros_flightPlans_sub
        ros_flightPlans_msg

        SimulationInput
    end
    
    methods
        function obj = Drone(model, initLoc)
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
        function obj = subToTelemety(obj)
            pause(1);
            obj.ros_Telemetry_sub = rossubscriber(sprintf('/drone/%d/odometry', obj.droneId));
            obj.timer_Upd_Telemetry = timer("Period", 5, "TimerFcn", @obj.updateLoc,"ExecutionMode","fixedRate");
            start(obj.timer_Upd_Telemetry);
        end

        %Subcription to the drone flight plan response 
        function obj = pubsubToFlightPlan(obj)
            rospublisher(sprintf('/drone/%d/flightPlans/response', obj.droneId), "siam_main/FlightPlan");
            obj.ros_flightPlans_pub = rospublisher(sprintf('/drone/%d/flightPlans/request', obj.droneId), "siam_main/FlightPlan");
            obj.ros_flightPlans_sub = rossubscriber(sprintf('/drone/%d/flightPlans/response', obj.droneId));
            obj.timer_Upd_Status = timer("Period", 5, "TimerFcn", @obj.updateStatus,"ExecutionMode","fixedRate");
            start(obj.timer_Upd_Status);
        end

        %Callback to update location of the drone
        function obj = updateLoc(obj, timer, time)
            [obj.ros_Telemetry_msg, status] = receive(obj.ros_Telemetry_sub, 0.2);
            if status
                obj.loc = [obj.ros_Telemetry_msg.Pose.Position.X obj.ros_Telemetry_msg.Pose.Position.Y obj.ros_Telemetry_msg.Pose.Position.Z];
            end
        end

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
    end

end

