%Configuracion de la simulacion%

%Numero de drones en la simulacion
num_drone = 4;

%Nombre del modelo de simulink
model_name = "drone_control_tut3_R22a";
%Nombre de los topicos
pub_bus_command = model_name + "/drone simulator/command bus/pub_bus_command";
sub_camera = model_name + "/drone simulator/camera bus/ROS communication/sub_camera";
sub_odometry = model_name + "/drone simulator/imu bus/sub_odometry";


%Apertura del modelo (para tenerlo cargado)
open_system(model_name);

%Puesta en ejecucion de los controladores para los UAV
for i = 0:1:(num_drone-1)
    %Se genera una configuracion nueva de parametros de entrada del modelo
    model_config(i+1) = Simulink.SimulationInput(model_name);

    %Modificacion de los nombres de los topicos en el modelo
    model_config(i+1) = model_config(i+1).setBlockParameter(pub_bus_command, "Topic", "/drone/"+i+"/bus_command");
    model_config(i+1) = model_config(i+1).setBlockParameter(sub_camera, "Topic", "/drone/"+i+"/onboard_camera/image_raw");
    model_config(i+1) = model_config(i+1).setBlockParameter(sub_odometry, "Topic", "/drone/"+i+"/odometry");
end

%Ejecucion de la simulacion de forma paralela
out = parsim(model_config);
