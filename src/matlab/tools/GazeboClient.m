classdef GazeboClient
    properties
        ROSnode
        ROSsub_clock  % Suscriptor para el reloj
        ROScli_pause  % Cliente de servicio para pausar la simulación
        ROScli_play   % Cliente de servicio para reanudar la simulación
        ROScli_reset  % Cliente de servicio para reiniciar la simulación
    end
    
    methods
        % Constructor
        function obj = GazeboClient()
            % Inicializar la conexión ROS
            obj.ROSnode = ros2node('matlab_node');

            % Suscribirse al reloj de Gazebo
            obj.ROSsub_clock = ros2subscriber(obj.ROSnode, ...
                '/clock','rosgraph_msgs/Clock', ...
                'Reliability','besteffort', ...
                'Durability','volatile', ...
                'Depth',1);

            % Crear clientes de servicio
            obj.ROScli_pause = ros2svcclient(obj.ROSnode, ...
                '/pause_physics','std_srvs/Empty');
            obj.ROScli_play  = ros2svcclient(obj.ROSnode, ...
                '/unpause_physics','std_srvs/Empty');
            obj.ROScli_reset = ros2svcclient(obj.ROSnode, ...
                '/reset_simulation','std_srvs/Empty');

        end


        % Obtener la hora del sistema simulado
        function secs = getSimulationTime(obj)
            [msg,status,statustext] = receive(obj.ROSsub_clock,0.1);  
            secs = msg.clock.sec;
        end


        % Pausar la simulación
        function pauseSimulation(obj)
            call(obj.ROScli_pause);
        end

        % Reanudar la simulación
        function playSimulation(obj)
            call(obj.ROScli_play);
        end

        % Reiniciar la simulación en tiempo 0
        function resetSimulation(obj)
            call(obj.ROScli_reset);
        end

      end
end
