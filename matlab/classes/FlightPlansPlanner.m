%Entity responsible of sending flight plans to the drones when is time to
%execute the flight plan

classdef FlightPlansPlanner < handle
    
    properties
        UTM
        timer_flightPlansExecuter       %Timer object reference
        lastExecuted_flightPlanId uint32 = 1;  %Int: Last FP id executed
    end
    
    methods
        %Constructor of the class
        function obj = FlightPlansPlanner(UTM)
           obj.UTM = UTM;
           obj.timer_flightPlansExecuter = timer("TimerFcn", @obj.fpsScheduler,"ExecutionMode","fixedRate","Period",1);
           start(obj.timer_flightPlansExecuter);
        end

        function obj = fpsScheduler(obj, timer, time)
            %If GClock value is not initialised, discard 
            if isempty(obj.UTM.Gclock)
                return;
            end

            time = obj.UTM.Gclock;
            fps = obj.UTM.S_Registry.flightPlans;

            %If no flight plan scheduled
            if isempty(fps)
                return;
            end

            i = 1;
            while i <= size(fps,2) && (fps(i).dtto-5) < time
                fp = fps(i);
                %If Uplan has finished, continue
                if fp.status == 2
                    i = i+1;
                    continue;
                end

                %If Uplan has been sent, continue
                if fp.sent
                    i = i+1;
                    continue;
                end

                drone = obj.UTM.S_Registry.drones(fp.drone.droneId);
                
                %Assign flightPlan to the drone
                if isempty(drone.flightPlan)
                    drone.flightPlan = fp;
                end

                %Sent flight plan to the drone through the topic
                send(drone.ros_flightPlans_pub,fp.parseToROSMessage())
                pause(.2);
                fp.sent = 1;
                i = i+1;
            end
        end
    end
end

