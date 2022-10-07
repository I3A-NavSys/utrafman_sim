
classdef UTMAirspace < handle
    
    properties
        %Entidades necesarias en el espacio aéreo

        %Servicio de registro de entidades
        S_Registry = Registry();

        %Servicio de planificacion de planes de vuelo
        S_FlightPlansPlanner = FlightPlansPlanner();

        %Servicio de monitorizacion de vuelos
        S_FlightPlansMonitor;
        
    end
    
    methods
        function obj = UTMAirspace()
        end
    end
end

