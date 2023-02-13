%Class for a UAV in the simulation

classdef Drone < handle

    properties
        UTM                         %Object reference to the airspace

        droneId uint32              %Unique ID of the drone
        model string                %TODO: (rename or redifine): Drone model 
        operator Operator           %Operator of the drone

        flightPlan FlightPlan       %FlightPlan object reference
        status int8                 %Status flag

        initLoc                     %Vector3<double>: Spawn location of the drone in the world 
        loc(1,3) double             %Current location of the drone
        locs = ros.msggen.utrafman_main.Telemetry.empty(0); %Array of Vector3<Float>

        droneModel string
        sdf string                  %String: SDF definition of the drone model

        %ROS
        ros_Telemetry_sub       %ROS Subscriber object reference
        ros_flightPlans_pub     %ROS Publisher object reference 
    end
    
    methods
        %Constructor of the class
        function obj = Drone(utm, model, initLoc)
            obj.UTM = utm;
            obj.model = model;
            obj.initLoc = initLoc;
        end

        %Generate SDF model to be used in Gazebo
        function obj = generateSDF(obj)
            obj.selectModel();
            obj.sdf = sprintf(obj.droneModel, obj.droneId, obj.initLoc(1), obj.initLoc(2), obj.initLoc(3), obj.droneId, obj.droneId, obj.droneId);
        end

        %Select Gazebo model to insert
        %This will in the future add new drone models 
        function obj = selectModel(obj)
            switch obj.model
                otherwise
                    obj.droneModel = fileread('../gazebo-ros/src/utrafman_main/models/drone2/template_model_2_simple.sdf');
            end
        end 

        %Subcription to the drone odometry topic
        function obj = subToTelemety(obj, timer, time)
            %Subscription with callback
            obj.ros_Telemetry_sub = rossubscriber(sprintf('/drone/%d/telemetry', obj.droneId),"utrafman_main/Telemetry",@obj.updateLoc);
        end

        %Publisher for drone flight plans 
        function obj = pubsubToFlightPlan(obj, timer, time)
            obj.ros_flightPlans_pub = rospublisher(sprintf('/drone/%d/uplan', obj.droneId),"utrafman_main/Uplan");
        end

        %Callback to update location of the drone with message received
        %through the topic
        function updateLoc(obj, sub, msg)
            obj.loc = [msg.Pose.Position.X msg.Pose.Position.Y msg.Pose.Position.Z];
            obj.locs(end+1) = msg;
        end

        %Not in use - TODO: reimplement
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

        %To remove models from the world
        function obj = removeDrone(obj)
            % First: send a topic message to the drone
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
        end

        %Return drone telemetry between two times
        function tel = filterTelemetryByTime(drone, ti, tf)
            first = 1; last = size(drone.locs,2);
            for i=1:length(drone.locs)
                if (drone.locs(i).Time.Sec <= ti)
                    first = i;
                end
                if (drone.locs(i).Time.Sec <= tf)
                    last = i;
                end
            end
            tel = drone.locs(first:last);
        end
    end

end

