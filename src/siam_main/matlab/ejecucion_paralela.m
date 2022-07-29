%Configuracion de la simulacion%

model_name = "drone_control_tut3_R22a";
pub_bus_command = model_name + "/drone simulator/command bus/pub_bus_command";
sub_camera = model_name + "/drone simulator/camera bus/ROS communication/sub_camera";
sub_odometry = model_name + "/drone simulator/imu bus/sub_odometry";

num_drone = 4;


% modelo = load_system(model_name);
% model = getActiveConfigSet(model_name);

open_system(model_name);

% get_param(sub_odometry, "Topic")

for i = 0:1:(num_drone-1)
    model_config(i+1) = Simulink.SimulationInput(model_name);
    model_config(i+1) = model_config(i+1).setBlockParameter(pub_bus_command, "Topic", "/drone/"+i+"/bus_command");
    model_config(i+1) = model_config(i+1).setBlockParameter(sub_camera, "Topic", "/drone/"+i+"/onboard_camera/image_raw");
    model_config(i+1) = model_config(i+1).setBlockParameter(sub_odometry, "Topic", "/drone/"+i+"/odometry");
end

out = parsim(model_config);
