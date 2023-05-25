%FlightPlansPlanner class takes care of sending flight plans to the drones when is time to execute the flight plan. 
%It is a timer that is executed every second and checks if there is any flight plan to be executed. If there is, it sends the flight plan to the drone through the topic.
classdef FlightPlansPlanner < handle
    
    properties
        UTM                                         %UTMAirspace object reference   
        timer_flightPlansExecuter                   %Timer to execute flight plans
        lastExecuted_flightPlanId uint32 = 1;       %Last FP id executed
    end
    
    methods
        %Constructor of the class
        function obj = FlightPlansPlanner(UTM)
           obj.UTM = UTM;
           %Create timer to execute flight plans
           obj.timer_flightPlansExecuter = timer("TimerFcn", @obj.fpsScheduler,"ExecutionMode","fixedRate","Period",1);
           %Start timer
           start(obj.timer_flightPlansExecuter);
        end

        function obj = fpsScheduler(obj, timer, time)
            %If GClock value is not initialised, discard execution of the timer
            if isempty(obj.UTM.Gclock)
                return;
            end

            time = obj.UTM.Gclock;
            fps = obj.UTM.S_Registry.flightPlans;

            %Discard execution of the timer if there is no flight plans in the registry
            if isempty(fps)
                return;
            end

            i = 1;
            %Iterate through all the flight plans in the registry
            %TBD: Check conditions
            while i <= size(fps,2) && (fps(i).dtto-5) < time
                fp = fps(i);

                %If flight plan is already executed, continue
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
                fp.sent = 1;
                i = i+1;
            end
        end
    end
end

