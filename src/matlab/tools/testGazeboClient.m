% Crear instancia del cliente Gazebo

%gazeboClient = GazeboClient('192.168.17.128');
gazeboClient = GazeboClient('192.168.225.128');

% Consultar la hora del sistema simulado
simulationTime = gazeboClient.getSimulationTime();
disp(['Hora del sistema simulado: ' num2str(simulationTime)]);

% Pausar la simulación
gazeboClient.pauseSimulation();

% Hacer algo mientras la simulación está pausada...

% Reanudar la simulación
gazeboClient.playSimulation();

% Reiniciar la simulación en tiempo 0
gazeboClient.resetSimulation();

% Liberar recursos al finalizar
delete(gazeboClient);
