%Calculo del error para un onjunto completo de simulacion

%Sim properties
    % 1-> Tiempo 
    % 2-> Waypoints
    % 3-> Precision
simProperties = zeros(length(UTM.S_Registry.flightPlans),3);

%Almacenamiento del error
    % 1-> Error total acumulado
    % 2-> Minimo
    % 3-> Maximo
error = zeros(length(UTM.S_Registry.flightPlans),2);

%Por cada plan de vuelo
for j = 1:length(UTM.S_Registry.flightPlans)
    %UPlan bajo analisis
    Uplan = UTM.S_Registry.flightPlans(j);
    
    %Obtenemos el tiempo de incio del Uplan
    inicio = Uplan.dtto;
    final = Uplan.route(end).T.Sec;

    %Sim properties
    simProperties(j,1) = final-inicio;
    simProperties(j,2) = length(Uplan.route);

    %Telemetria del drone
    droneTelemetry = UTM.S_Registry.flightPlans(j).drone.filterTelemetryByTime(inicio, final);
    
    %Maximos y minimos
    error(j,3) = 0;

    %Analizamos cada mensaje de telemetria
    % (la precision del error ira marcada por la resolucion de los mensajes
    % de telemetria)
    for i=1:length(droneTelemetry)
        %Posicion real y posicion de referencia
        t = droneTelemetry(i).Time.Sec + droneTelemetry(i).Time.Nsec*10e-10;
        reference = Uplan.AbstractionLayer(t);
        real = [droneTelemetry(i).Pose.Position.X droneTelemetry(i).Pose.Position.Y droneTelemetry(i).Pose.Position.Z];
        err = norm(reference-real);
        %Error acumulado
        error(j,1) = error(j,1) + err;

        %Maximos y minimos
        %Seteo del primer minimo
        if i == 1
            error(j,2) = norm(reference-real);
        end
        
        %Nuevo minimo
        if error(j,2) > err
            error(j,2) = err;
        end

        %Nuevo maximo
        if error(j,3) < err
            error(j,3) = err;
        end
    end

    simProperties(j,3) = simProperties(j,1)/i;
end

figure;
%Organizacion y redondeo de los datos
data = [error(:,2) error(:,1)./simProperties(:,1) error(:,3)];
data = round(data,3);
bar(data);
%Limites de la grafica
ylimits = [0 2];
ylim(ylimits);
%Obtencion de la posicion en X de las etiquetas
disp = 0.22;
x1 = (1:length(data(:,1)')) - disp;
x2 = 1:length(data(:,2)');
x3 = (1:length(data(:,3)')) + disp;
%Copiamos los datos para obtener la posicion en Y de las etiquetas
data2 = data;
data2(data2 > ylimits(2)) = ylimits(2);
data2(data2 < ylimits(1)) = ylimits(1); 
%Creacion de las etiquetas
text(x1,data2(:,1)',num2str(data(:,1)),'vert','bottom','horiz','center');
text(x2,data2(:,2)',num2str(data(:,2)),'vert','bottom','horiz','center');
text(x3,data2(:,3)',num2str(data(:,3)),'vert','bottom','horiz','center');
box off
%Restto de datos de las gráficas
legend(["Min" "Mean" "Max"]);
xlabel("Drone ID in the simulation");
ylabel("Error (m) with the reference per second");
grid on;