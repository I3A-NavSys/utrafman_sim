%This file is used to compute the error between U-plan and simulation route
%traveled by an aircraft. With it, the autopilot software could be
%evaluated measuring the error produced between the commanded route and the
%traveled one.

addpath("classes\");
uav_ids = [SP.getUavById(0).Id];
uplan_id = 0;

%Simulation propierties
    % 1-> Uplan total time (end-init)
    % 2-> Number of Waypoints
    % 3-> Precission (not used)
simProperties = zeros(length(uav_ids),3);

%Error variable
    % 1-> Total cumulative error
    % 2-> Minimum error
    % 3-> Maximum error
error2 = zeros(length(uav_ids),2);

%For each U-plan in the entire simulation
uplans = SP.getFpById(uplan_id);
for j = 1:length(uplans)
    %Take the U-plan
    uplan = uplans(j);
    
    %Take the init and final timie of the U-plan
    t_init = uplan.Dtto;
    t_end = uplan.Route(end).T.Sec;

    %U-plan properties
    simProperties(j,1) = t_end-t_init;
    simProperties(j,2) = length(uplan.Route);

    %UAV telemetry
    uavTel = SP.filterUavTelemetryByTime(SP.getUavTelemetry(uplan.DroneId), t_init, t_end);

    %Numero de drone
    %droneIDs(end+1) = UTM.S_Registry.flightPlans(j).drone.droneId;
    
    %Max and min
    error2(j,3) = 0;

    %For each telemetry sent by the UAV 
    % (precission of the error is determined by the sampling time)
    for i=1:height(uavTel)
        %Compute difference between U-plan position and simulation UAV position
        t = uavTel{i, "Time"};
        reference = FlightPlanProperties.abstractionLayerUplan(uplan, t);
        real = [uavTel{i,"PositionX"} uavTel{i,"PositionY"} uavTel{i,"PositionZ"}];
        err = norm(reference-real);
        %Cumulative error
        error2(j,1) = error2(j,1) + err;

        %Max and min
        %Set the first min
        if i == 1
            error2(j,2) = norm(reference-real);
        end
        
        %If new min is found
        if error2(j,2) > err
            error2(j,2) = err;
        end

        %If new max is found
        if error2(j,3) < err
            error2(j,3) = err;
        end
    end

    simProperties(j,3) = simProperties(j,1)/i;
end

%Data rounding and organization
figure('Position',[0 100 1300 500]);

data = [error2(:,2) error2(:,1)./simProperties(:,1) error2(:,3)];
data = round(data,3);
bar(data);

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