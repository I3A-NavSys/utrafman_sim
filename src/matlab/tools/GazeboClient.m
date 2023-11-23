classdef GazeboClient
    properties
        ROSsub_clock  % Suscriptor para el reloj
        ROScli_pause  % Cliente de servicio para pausar la simulación
        ROScli_play   % Cliente de servicio para reanudar la simulación
        ROScli_reset  % Cliente de servicio para reiniciar la simulación
    end
    
    methods
        % Constructor
        function obj = GazeboClient(ipAddress)
            % Inicializar la conexión ROS
            rosshutdown
            rosinit(ipAddress);

            % Suscribirse al reloj de Gazebo
            obj.ROSsub_clock = rossubscriber("/clock","rosgraph_msgs/Clock","DataFormat","struct");

            % Crear clientes de servicio
            % obj.ROScli_pause  = rossvcclient("/gazebo/pause_physics", "std_srvs/Empty","DataFormat","struct");
            % obj.ROScli_play   = rossvcclient("/gazebo/unpause_physics", "std_srvs/Empty","DataFormat","struct");
            % obj.ROScli_reset  = rossvcclient("/gazebo/reset_simulation", "std_srvs/Empty","DataFormat","struct");

        end

        % Obtener la hora del sistema simulado
        function time = getSimulationTime(obj)
            clockData = receive(obj.ROSsub_clock, 5);  % Recibir datos del reloj
            time = clockData.Clock_.Sec;
        end

        % Pausar la simulación
        function pauseSimulation(obj)
            call(obj.ROScli_pause, rosmessage(obj.ROScli_pause));
        end

        % Reanudar la simulación
        function playSimulation(obj)
            call(obj.ROScli_play, rosmessage(obj.ROScli_play));
        end

        % Reiniciar la simulación en tiempo 0
        function resetSimulation(obj)
            call(obj.ROScli_reset, rosmessage(obj.ROScli_reset));
        end

        % Destructor
        function delete(obj)
            % Cerrar la conexión ROS al finalizar
            rosshutdown;
        end
    end
end
