classdef Drone < handle

    properties
        droneId
        model
        operator

        flightPlan

        initLoc
        loc

        droneModel
        sdf

        timer_Upd_Telemetry
        ros_Telemetry_sub
        ros_Telemetry_msg
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
                    obj.droneModel = fileread('../../models/dronechallenge_models/drone/model_template_1_simple.sdf');
            end
        end

        function obj = subToTelemety(obj)
            pause(1);
            obj.ros_Telemetry_sub = rossubscriber(sprintf('/drone/%d/odometry', obj.droneId));
            obj.timer_Upd_Telemetry = timer("Period", 5, "TimerFcn", @obj.updateLoc,"ExecutionMode","fixedRate");
            start(obj.timer_Upd_Telemetry);
        end

        function obj = updateLoc(obj, timer, time)
            [obj.ros_Telemetry_msg, status] = receive(obj.ros_Telemetry_sub, 0.2);
            if status
                obj.loc = [obj.ros_Telemetry_msg.Pose.Position.X obj.ros_Telemetry_msg.Pose.Position.Y obj.ros_Telemetry_msg.Pose.Position.Z];
            end
        end
    end

end

