%This file is used to compute the error between U-plan and simulation route
%traveled by an aircraft. With it, the autopilot software could be
%evaluated measuring the error produced between the commanded route and the
%traveled one.

%If you want to load data from previous simulations, use this!
% load simulations\sim20.mat
% name = 'sim20';

%Simulation propierties
    % 1-> Time 
    % 2-> Waypoints
    % 3-> Precission
simProperties = zeros(length(UTM.S_Registry.flightPlans),3);

%Errors
    % 1-> Total cumulative error
    % 2-> Minimum error
    % 3-> Maximum error
error = zeros(length(UTM.S_Registry.flightPlans),2);

%For each U-plan in the entire simulation
for j = 1:length(UTM.S_Registry.flightPlans)
    %Take the U-plan
    Uplan = UTM.S_Registry.flightPlans(j);
    
    %Take the init and final timie of the U-plan
    inicio = Uplan.dtto;
    final = Uplan.route(end).T.Sec;

    %U-plan properties
    simProperties(j,1) = final-inicio;
    simProperties(j,2) = length(Uplan.route);

    %Drone telemetry
    droneTelemetry = UTM.S_Registry.flightPlans(j).drone.filterTelemetryByTime(inicio, final);
    
    %Max and min
    error(j,3) = 0;

    %For each telemetry sent by the drone 
    % (precission of the error is determined by the sampling time)
    for i=1:length(droneTelemetry)
        %Compute difference between U-plan position and simulation drone
        %position
        t = droneTelemetry(i).Time.Sec + droneTelemetry(i).Time.Nsec*10e-10;
        reference = Uplan.AbstractionLayer(t);
        real = [droneTelemetry(i).Pose.Position.X droneTelemetry(i).Pose.Position.Y droneTelemetry(i).Pose.Position.Z];
        err = norm(reference-real);
        %Cumulative error
        error(j,1) = error(j,1) + err;

        %Max and min
        %Set the first min
        if i == 1
            error(j,2) = norm(reference-real);
        end
        
        %If new min is found
        if error(j,2) > err
            error(j,2) = err;
        end

        %If new max is found
        if error(j,3) < err
            error(j,3) = err;
        end
    end

    simProperties(j,3) = simProperties(j,1)/i;
end

%Data rounding and organization
figure('Position',[0 100 1300 500]);
data = [error(:,2) error(:,1)./simProperties(:,1) error(:,3)];
data = round(data,3);
bar(data);
title(name);

%Figure limits
ylimits = [0 2];
ylim(ylimits);

%X position in the figure of the tags
disp = 0.22;
x1 = (1:length(data(:,1)')) - disp;
x2 = 1:length(data(:,2)');
x3 = (1:length(data(:,3)')) + disp;

%Y position in the figure of the tags
data2 = data;
data2(data2 > ylimits(2)) = ylimits(2);
data2(data2 < ylimits(1)) = ylimits(1);

%Tag at side
text(x1,data2(:,1)',num2str(data(:,1)),'vert','bottom','horiz','center');
text(x2,data2(:,2)',num2str(data(:,2)),'vert','bottom','horiz','center');
text(x3,data2(:,3)',num2str(data(:,3)),'vert','bottom','horiz','center');

%Print means at side
text(j+0.7, ylimits(2),'Min mean:', 'vert','bottom','horiz','left');
text(j+0.7, ylimits(2)-0.1,num2str(mean(data(:,1))), 'vert','bottom','horiz','left');

text(j+0.7, ylimits(2)-0.3,'Total mean:', 'vert','bottom','horiz','left');
text(j+0.7, ylimits(2)-0.4,num2str(mean(data(:,2))), 'vert','bottom','horiz','left');

text(j+0.7, ylimits(2)-0.6,'Max mean:', 'vert','bottom','horiz','left');
text(j+0.7, ylimits(2)-0.7,num2str(mean(data(:,3))), 'vert','bottom','horiz','left');
box off

%Tag values
legend(["Min" "Mean" "Max"]);
xlabel("Drone ID in the simulation");
ylabel("Error (m) with the reference per second");
grid on;