clc
clear


gz = GazeboClient;

simulationTime = gz.getSimulationTime();

disp(['Hora del sistema simulado: ' num2str(simulationTime)]);

% Pausar la simulación
gz.pauseSimulation();

% Hacer algo mientras la simulación está pausada...

% Reanudar la simulación
gz.playSimulation();

% Reiniciar la simulación en tiempo 0
gz.resetSimulation();


