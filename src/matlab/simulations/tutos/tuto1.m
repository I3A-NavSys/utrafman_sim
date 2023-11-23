% Tutorial:


% PRE-SIMULATION TASKs
clc; clear;
run('../../tools/UTRAFMAN_init');

%-----------------------------------------


gz = GazeboConnector(ROS_MASTER_IP);
gz.pause();
pause(1)
gz.reset();


reg = USpace_registrator(gz);

% monitor = S_Monitoring();
% monitor.execute(ROS_MASTER_IP);

%-----------------------------------------
%ROUTE
route_3d = [

         0     0    3
         0     5    3
         5     5    3
         5     0    3   
         0     0    3
         0     0    0
];

pos = [0 0 1];


% Registering an operator
operator = SimpleOperator(gz,'SimpleOperator');

% Create and register a new UAV
uav = UAVProperties.empty;
uav = operator.regNewDrone("abejorro", pos);


%Uplan generation
fp = FlightPlanProperties.empty();                %FlightPlan instance
fp = FlightPlanProperties(operator, ...          %Operator
                   uav, ...                     %UAV asignation
                   route_3d, ...                     %Route
                   gz.timeSeconds + 10);       %DTTO (desired time to take off)

%FP registration
operator.regNewFP(fp);

%Sent FP to UAV
operator.sendFlightPlan(fp);

gz.play;



