%Para funcionar en un entorno distribuido, deben estar configuradas ambas
%máquinas en los ficheros hosts (/etc/hosts). Enlaces de ayuda:
% https://answers.ros.org/question/90536/ros-remote-master-can-see-topics-but-no-data/
% http://wiki.ros.org/ROS/NetworkSetup

%Configuracion del master de ROS y conexion
ros_master_ip = "192.168.2.111";
ros_master_port = 11311;

scheduleSimulation(ros_master_ip, 1, 1);

%Inicio de la conexion con el master
try
    rosinit(ros_master_ip, ros_master_port);
catch
    disp("ROS ya está iniciado. Reiniciando la conexión ...");
    rosshutdown;
    rosinit(ros_master_ip, ros_master_port);
end
%Creacion del suscriptor para insertar los drones
[inserter_publisher, ros_msg] = rospublisher("/god/insert");


%Configuracion de la simulacion%

%Numero de drones en la simulacion
num_drone = 2;
drone_used_positions = zeros(9,9);
entrada_usuario = 1; %Numero indicando drones controlados por usuarios existen

%Modelo de los drones extraido de un fichero
drone_model = fileread('../models/dronechallenge_models/drone/model_template_1.sdf');
user_drone_model = fileread('../models/dronechallenge_models/drone/model_template_2.sdf');

%Insercion de los modelos en el mundo
for i = 0:1:(num_drone-1)
    drone_id = i;

    xy = randi([-4, 4], 1, 2);
    while drone_used_positions(xy(1)+5,xy(2)+5) == 1
        xy = randi([-4, 4], 1, 2);
    end
    
    drone_used_positions(xy(1)+5,xy(2)+5) = 1;   
    drone_init_pos = [xy(1) xy(2) 1];
    %Generacion de la definicion de un drone
    if i<entrada_usuario
        drone_def = sprintf(user_drone_model, drone_id, drone_init_pos(1), drone_init_pos(2), drone_init_pos(3), drone_id, drone_id, drone_id);
    else
        drone_def = sprintf(drone_model, drone_id, drone_init_pos(1), drone_init_pos(2), drone_init_pos(3), drone_id, drone_id, drone_id);
    end
    ros_msg.Data = drone_def;
    %Envio del mensaje por el topico
    send(inserter_publisher, ros_msg);
    pause(1);
end

%Nombre del modelo de simulink
model_name = "drone_control_tut3_R22a";
%Nombre de los topicos
pub_bus_command = model_name + "/drone simulator/command bus/pub_bus_command";
sub_camera = model_name + "/drone simulator/camera bus/ROS communication/sub_camera";
sub_odometry = model_name + "/drone simulator/imu bus/sub_odometry";


%Apertura del modelo (para tenerlo cargado)
open_system(model_name);

%Puesta en ejecucion de los controladores para los UAV
for i = entrada_usuario:1:(num_drone-1)
    %Se genera una configuracion nueva de parametros de entrada del modelo
    model_config(i-entrada_usuario+1) = Simulink.SimulationInput(model_name);

    %Modificacion de los nombres de los topicos en el modelo
    model_config(i-entrada_usuario+1) = model_config(i-entrada_usuario+1).setBlockParameter(pub_bus_command, "Topic", "/drone/"+i+"/bus_command");
    model_config(i-entrada_usuario+1) = model_config(i-entrada_usuario+1).setBlockParameter(sub_camera, "Topic", "/drone/"+i+"/onboard_camera/image_raw");
    model_config(i-entrada_usuario+1) = model_config(i-entrada_usuario+1).setBlockParameter(sub_odometry, "Topic", "/drone/"+i+"/odometry");
end

%Ejecucion de la simulacion de forma paralela
out = parsim(model_config);

function scheduleSimulation(ros_master_ip, simulation_scheduled, stop_simulation)

request = matlab.net.http.RequestMessage;
uri_deploy = matlab.net.URI(sprintf('http://%s:3000/deploy/dronechallenge.launch', ros_master_ip));
uri_destroy = matlab.net.URI(sprintf('http://%s:3000/destroy', ros_master_ip));

while simulation_scheduled
    response = send(request, uri_deploy);
    if (response.StatusCode == 200)
        disp(response.Body.Data);
        simulation_scheduled = 0;
        pause(3);
    else
        if ~stop_simulation
            disp('Previous simuation is running. Stopping it ... ');
            response = send(request, uri_destroy);
            pause(5);
        else
            disp('Stopping simulation.');
            simulation_scheduled = 0;
            response = send(request, uri_destroy);
            disp(response.Body.Data);
        end
    end
end
end
