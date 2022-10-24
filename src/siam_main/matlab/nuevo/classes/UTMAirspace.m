
classdef UTMAirspace < handle
    
    properties
        %Entidades necesarias en el espacio aéreo

        %Servicio de registro de entidades
        S_Registry;

        %Servicio de planificacion de planes de vuelo
        S_FlightPlansPlanner = FlightPlansPlanner();

        %Servicio de monitorizacion de vuelos
        S_FlightPlansMonitor;
        
        %ROS info
        rosMasterIp = "1.0.0.131";
        rosMasterPort = 11311;

        GClock_sub;
        GClock_Upt_timer;
        Gclock;
    end
    
    methods
        function obj = UTMAirspace()
            obj.ConnectWithROSMaster();
            obj.S_Registry = Registry();
        end

        function ConnectWithROSMaster(obj)
            try
                rosinit(obj.rosMasterIp, obj.rosMasterPort);
            catch
                disp("ROS ya está iniciado. Reiniciando conexión...");
                rosshutdown;
                rosinit(obj.rosMasterIp, obj.rosMasterPort);
            end

            %Suscribe to the clock
            obj.GClock_sub = rossubscriber("/clock");
            %Added timer to update GClock every 1sec
            obj.GClock_Upt_timer = timer("TimerFcn", @obj.updateGClock,"ExecutionMode","fixedRate");
            start(obj.GClock_Upt_timer);
        end
        
        function obj = updateGClock(obj, timer, time)
            [msg, status] = receive(obj.GClock_sub, 0.1);
            if status
                obj.Gclock = msg.Clock_.Sec;
            end
        end
    end
end

