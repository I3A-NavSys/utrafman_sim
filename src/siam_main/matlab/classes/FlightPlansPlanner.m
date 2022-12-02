%Entidad que planifica los vuelos en el espacio aéreo, correspondiente con
%el bloque "Drone Operation Planning"

classdef FlightPlansPlanner < handle
    
    properties
        UTM
        timer_flightPlansExecuter
        lastExecuted_flightPlanId = 1;
    end
    
    methods
        function obj = FlightPlansPlanner(UTM)
           obj.UTM = UTM;
           obj.timer_flightPlansExecuter = timer("TimerFcn", @obj.fpsScheduler,"ExecutionMode","fixedRate","Period",3);
           start(obj.timer_flightPlansExecuter);
        end

        function obj = fpsScheduler(obj, timer, time)
            %If no Gclock, return
            if isempty(obj.UTM.Gclock)
                return;
            end

            time = obj.UTM.Gclock;
            fps = obj.UTM.S_Registry.flightPlans;

            %If no fps, return
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
                
                if isempty(drone.flightPlan)
                    %Asignamos el plan de vuelo
                    drone.flightPlan = fp;
                end

                %Lo enviamos por el topico
                send(drone.ros_flightPlans_pub,fp.parseToROSMessage())
                pause(.2);
                fp.sent = 1; %Mark as sent
                i = i+1;
            end

        end
    end
end

