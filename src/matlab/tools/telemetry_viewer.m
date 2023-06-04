%This file is used to analyse flight plans and simulations. In the viewer,
%you can see the commanded U-plan and the real followed trayectory traveled
%by the aircraft. 

%--------------------------------------------------------------------------

%Added classes to the path
addpath("./classes/");
addpath('.')

%If you want to load data from previous simulations, use this!
%load simulations\small_city_LaLlanura.mat

%Introduce drone ID and FlightPlan ID to be analysed
fpId = 1;

%Get ROS IP
% run(fullfile("./config/ros.m"));

%Connect to ROS
% rosnode = ros.Node('telemetry_viewer',ROS_IP, 11311);
% ros_get_telemetry = ros.ServiceClient(rosnode,'/service/monitoring/get_telemetry');
% ros_get_flightplan = ros.ServiceClient(rosnode,'/service/registry/get_fps');
% 
% msg = ros_get_telemetry.rosmessage;
% msg.UavId = uavId;
% if isServerAvailable(ros_get_telemetry)
%     telemetry = call(ros_get_telemetry,msg,"Timeout",100);
% else
%     error("Service '/service/monitoring/get_telemetry' not available on network");
% end
% drone.locs = telemetry.Telemetry;
% 
% msg = ros_get_flightplan.rosmessage;
% msg.FpId = fpId;
% if isServerAvailable(ros_get_flightplan)
%     flightplan = call(ros_get_flightplan,msg,"Timeout",5);
% else
%     error("Service '/service/monitoring/get_telemetry' not available on network");
% end
% uplan = flightplan.Fps;

%Using simulated data
figure(1);

fp_obj = SP.getFpById(fpId);
fp_wps = SP.getFpWaypoints(fp_obj);

uavId = fp_obj.DroneId;
uav_obj = SP.getUavById(uavId);
uav_tel = SP.getUavTelemetry(uav_obj);
uav_tel = SP.filterUavTelemetryByTime(uav_tel, min(fp_wps.Time), max(fp_wps.Time));


drone.initLoc = [uav_tel.PositionX(1) uav_tel.PositionY(1) uav_tel.PositionZ(1)];


%uplan = UTM.S_Registry.flightPlans(fp);
waypoints = [fp_wps.X, fp_wps.Y, fp_wps.Z];

%Allocating variables
% t = zeros(1,size(drone.locs',2));
% x = zeros(1,size(drone.locs',2));
% y = zeros(1,size(drone.locs',2));
% z = zeros(1,size(drone.locs',2));
% rotz = zeros(1,size(drone.locs',2));
% dx = zeros(1,size(drone.locs',2));
% dy = zeros(1,size(drone.locs',2));
% dz = zeros(1,size(drone.locs',2));
% drotz = zeros(1,size(drone.locs',2));
% 
% %Fill variables with simulation data
% for i=1:1:size(drone.locs',2)
%     tel = drone.locs(i);
%     t(i) = tel.Time.Sec;
%     x(i) = tel.Pose.Position.X;
%     y(i) = tel.Pose.Position.Y;
%     z(i) = tel.Pose.Position.Z;
%     rotz(i) = tel.Pose.Orientation.Z;
%     dx(i) = tel.Velocity.Linear.X;
%     dy(i) = tel.Velocity.Linear.Y;
%     dz(i) = tel.Velocity.Linear.Z;
%     drotz(i) = tel.Velocity.Angular.Z;
% end

t       = uav_tel.Time';
x       = uav_tel.PositionX';
y       = uav_tel.PositionY';
z       = uav_tel.PositionZ';
rotz    = uav_tel.OrientationZ';

dx      = uav_tel.VelLinX';
dy      = uav_tel.VelLinY';
dz      = uav_tel.VelLinZ';

%eul_trans = [cos(-uav_tel.OrientationZ) sin(-uav_tel.OrientationZ) 0 ; -sin(-uav_tel.OrientationZ) -cos(-uav_tel.OrientationZ) 0; 0 0 1] * [dx dy dz];
eul_trans = zeros(length(dx), 3);
for i=1:length(dx)
    mat = [cos(-uav_tel.OrientationZ(i)) sin(-uav_tel.OrientationZ(i)) 0 ; -sin(-uav_tel.OrientationZ(i)) -cos(-uav_tel.OrientationZ(i)) 0; 0 0 1];
    eul_trans(i,:) = mat * [dx(i); dy(i); dz(i)]; 
end

dx = eul_trans(:,1)';
dy = eul_trans(:,2)';
drotz   = uav_tel.VelAngZ';

%Generating data from Uplan
ut = zeros(1,length(fp_wps.Time)+2);

%Reference data
%Sim data recoding must be start before Uplan execution and end after Uplan
%finish. Reference data must start and end at the same time as simulation data to
%interpolate it correctly.

%Time
ut(1) = t(1);           %Init telemetry
ut(2) = fp_wps.Time(1);     %Init Uplan
for i=1:length(fp_wps.Time)
    ut(i+2) = fp_wps.Time(i);
end     
ut(i+3) = ut(i+2)+3;    %Finish uplan
ut(i+4) = t(end);       %Finish telemetry

%Positions
ux = [drone.initLoc(1) drone.initLoc(1) fp_wps.X' fp_wps.X(end) fp_wps.X(end)];
uy = [drone.initLoc(2) drone.initLoc(2) fp_wps.Y' fp_wps.Y(end) fp_wps.Y(end)];
uz = [0 0 fp_wps.Z' 0 0];

% dux(1) = 0; duy(1) = 0; duz(1) = 0;
% for i=2:size(uplan.route,2)
%     timeBetween = uplan.route(1,i).T.Sec - uplan.route(1,i-1).T.Sec;
%     dux(i) = (ux(i)-ux(i-1))/timeBetween;
%     duy(i) = (uy(i)-uy(i-1))/timeBetween;
%     duz(i) = (uz(i)-uz(i-1))/timeBetween;
% end
% dux(i+1) = 0; duy(i+1) = 0; duz(i+1) = 0;

%Generating datetimes of simulation and reference data
sim_dates = datetime(t,'ConvertFrom','epochtime','Epoch',0);
ref_dates = datetime(ut,'ConvertFrom','epochtime','Epoch',0);

% X Y Z of the simulation (first load data, then remove duplicates)
sim_x_timetable = timetable(sim_dates',x');
sim_x_timetable = retime(sim_x_timetable,unique(sim_x_timetable.Time),'mean');
sim_y_timetable = timetable(sim_dates',y');
sim_y_timetable = retime(sim_y_timetable,unique(sim_y_timetable.Time),'mean');
sim_z_timetable = timetable(sim_dates',z');
sim_z_timetable = retime(sim_z_timetable,unique(sim_z_timetable.Time),'mean');
sim_rotz_timetable = timetable(sim_dates',rotz');
sim_rotz_timetable = retime(sim_rotz_timetable,unique(sim_rotz_timetable.Time),'mean');

% dX dY dZ of the simulation
sim_dx_timetable = timetable(sim_dates',dx');
sim_dx_timetable = retime(sim_dx_timetable,unique(sim_dx_timetable.Time),'mean');
sim_dy_timetable = timetable(sim_dates',dy');
sim_dy_timetable = retime(sim_dy_timetable,unique(sim_dy_timetable.Time),'mean');
sim_dz_timetable = timetable(sim_dates',dz');
sim_dz_timetable = retime(sim_dz_timetable,unique(sim_dz_timetable.Time),'mean');
sim_drotz_timetable = timetable(sim_dates',drotz');
sim_drotz_timetable = retime(sim_drotz_timetable,unique(sim_drotz_timetable.Time),'mean');

% X Y Z of the reference (Uplan)
ref_x_timetable = timetable(ref_dates', ux');
ref_x_timetable = retime(ref_x_timetable,unique(ref_x_timetable.Time),'mean');
ref_y_timetable = timetable(ref_dates', uy');
ref_y_timetable = retime(ref_y_timetable,unique(ref_y_timetable.Time),'mean');
ref_z_timetable = timetable(ref_dates', uz');
ref_z_timetable = retime(ref_z_timetable,unique(ref_z_timetable.Time),'mean');

% dX dY dZ of the reference (Uplan)
%Interpolation of time
dut = [ut(1):1:ut(end)];
ref_dates_interpolated = datetime(dut,'ConvertFrom','epochtime','Epoch',0); %Generate time
ref_dates_temporal = timetable(ref_dates_interpolated');

%Interpolation of XYZ positions
ref_x_timetable = synchronize(ref_dates_temporal,ref_x_timetable,'first','linear');
ref_y_timetable = synchronize(ref_dates_temporal,ref_y_timetable,'first','linear');
ref_z_timetable = synchronize(ref_dates_temporal,ref_z_timetable,'first','linear');

%Generating XYZ velocities
ref_dx_timetable = timetable(ref_dates_interpolated(1:end-1)', diff(ref_x_timetable.Var1));
ref_dy_timetable = timetable(ref_dates_interpolated(1:end-1)', diff(ref_y_timetable.Var1));
ref_dz_timetable = timetable(ref_dates_interpolated(1:end-1)', diff(ref_z_timetable.Var1));

%Syncrionization between tables (sim and ref) and interpolation
x_timetable = synchronize(sim_x_timetable,ref_x_timetable, 'union', 'linear');
y_timetable = synchronize(sim_y_timetable,ref_y_timetable, 'union', 'linear');
z_timetable = synchronize(sim_z_timetable,ref_z_timetable, 'union', 'linear');
dx_timetable = synchronize(sim_dx_timetable,ref_dx_timetable, 'union', 'linear');
dy_timetable = synchronize(sim_dy_timetable,ref_dy_timetable, 'union', 'linear');
dz_timetable = synchronize(sim_dz_timetable,ref_dz_timetable, 'union', 'linear');
x_timetable = mergevars(x_timetable,[1 2],'NewVariableName','X');
y_timetable = mergevars(y_timetable,[1 2],'NewVariableName','Y');
z_timetable = mergevars(z_timetable,[1 2],'NewVariableName','Z');
dx_timetable = mergevars(dx_timetable,[1 2],'NewVariableName','X');
dy_timetable = mergevars(dy_timetable,[1 2],'NewVariableName','Y');
dz_timetable = mergevars(dz_timetable,[1 2],'NewVariableName','Z');

%3D viewer
subplot(6,2,[1 9]);
plot3(x,y,z);
grid on;
hold on;
plot3(ux(1:end-1),uy(1:end-1),uz(1:end-1));
plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),'or');
legend(["Simulated", "Reference", "Waypoints"],'Location','northwest');
title("Route 3D view")
xlabel("pos X");
ylabel("pos Y");
zlabel("pos Z");

max_uav = [max(uav_tel.PositionX) max(uav_tel.PositionY) max(uav_tel.PositionZ)];
min_uav = [min(uav_tel.PositionX) min(uav_tel.PositionY) min(uav_tel.PositionZ)];
max_fp = [max(fp_wps.X) max(fp_wps.Y) max(fp_wps.Z)];
min_fp = [min(fp_wps.X) min(fp_wps.Y) min(fp_wps.Z)];

xlim([min(min_uav(1),min_fp(1)) max(max_uav(1),max_fp(1))]);
ylim([min(min_uav(2),min_fp(2)) max(max_uav(2),max_fp(2))]);
zlim([min(min_uav(3),min_fp(3)) max(max_uav(3),max_fp(3))]);
view(45,30);
hold off;

%Bearing plot
subplot(6,2,11);
plot(sim_rotz_timetable, "Var1");
hold on;
grid on;
plot(sim_drotz_timetable, "Var1");
ylim([-pi,pi]);
xlabel("Time");
ylabel("Rot Z");
legend(["Position", "Velocity"]);
hold off;

%X,Y, Z pos
subplot(6,2,[2 6]);
possp = stackedplot([x_timetable, y_timetable, z_timetable]);
title("Positions through time");
xlabel('Simulated time');
grid on;
[possp.AxesProperties.LegendLabels] = deal({'Sim','Ref'},{'Sim','Ref'},{'Sim','Ref'});
[possp.AxesProperties.LegendLocation] = deal('southeast','southeast','southeast');

%X,Y, Z accel
subplot(6,2,[8 12]);
possp = stackedplot([dx_timetable, dy_timetable, dz_timetable]);
title("Velocities through time");
xlabel('Simulated time');
grid on;
[possp.AxesProperties.LegendLabels] = deal({'Sim','Ref'},{'Sim','Ref'},{'Sim','Ref'});
[possp.AxesProperties.LegendLocation] = deal('southeast','southeast','southeast');

drawnow ;

