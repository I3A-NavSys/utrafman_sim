%Configuracion del master de ROS y conexion
ros_master_ip = "192.168.2.111";
ros_master_port = 11311;

scheduleSimulation(1,0);

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
num_drone = 10;
drone_used_positions = zeros(9,9);

%Modelo de los drones
drone_model = "<?xml version='1.0' ?> <sdf version='1.5'> <model name='drone_%i'> <name>abejorro</name> <pose>%f %f %f 0 0 0</pose> <plugin name='DroneControl' filename='libDroneControl.so'> <id>%i</id> </plugin> <static>false</static> <link name='dronelink'> <!-- Rotores en distribucion cuadrada situados a 25 cms del centro de gravedad --> <visual name='fuselaje'> <pose>0 0 0 0 0 0</pose> <geometry> <box> <size>0.10 0.04 0.03</size> </box> </geometry> <material> <ambient>0.3 0.3 0.3 1</ambient> <diffuse>0.3 0.3 0.3 1</diffuse> </material> </visual> <collision name='fuselaje'> <pose>0 0 0 0 0 0</pose> <geometry> <box> <size>0.10 0.04 0.03</size> </box> </geometry> <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.2</mu2> <slip1>1.0</slip1> <slip2>1.0</slip2> </ode> </friction> </surface> </collision> <visual name='camara'> <pose>0.065 0 0 0 0 0</pose> <geometry> <box> <size>0.03 0.02 0.02</size> </box> </geometry> <material> <ambient>0.2 0.2 0.2 1</ambient> <diffuse>0.2 0.2 0.2 1</diffuse> </material> </visual> <collision name='camara'> <pose>0.065 0 0 0 0 0</pose> <geometry> <box> <size>0.03 0.02 0.02</size> </box> </geometry> <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.2</mu2> <slip1>1.0</slip1> <slip2>1.0</slip2> </ode> </friction> </surface> </collision> <sensor name='view_camera' type='camera'> <!-- <plugin name='onboard_camera' filename='/opt/ros/kinetic/share/drone_ros/plugins/build/libonboard_camera.so'> </plugin> --> <plugin name='camera_controller_2' filename='libgazebo_ros_camera.so'> <alwaysOn>true</alwaysOn> <updateRate>0.0</updateRate> <cameraName>view_camera</cameraName> <robotNamespace>drone/%i</robotNamespace> </plugin> <pose>-0.3 0 0.2 0 0.3 0</pose> <camera name='view_camera'> <horizontal_fov>1.57</horizontal_fov> <image> <width>320</width> <height>240</height> </image> <clip> <near>0.01</near> <far>20</far> </clip> </camera> <always_on>1</always_on> <update_rate>10</update_rate> <visualize>0</visualize> </sensor> <sensor name='onboard_camera_sensor' type='camera'> <!-- <plugin name='onboard_camera' filename='/opt/ros/kinetic/share/drone_ros/plugins/build/libonboard_camera.so'> </plugin> --> <plugin name='camera_controller' filename='libgazebo_ros_camera.so'> <alwaysOn>true</alwaysOn> <updateRate>0.0</updateRate> <cameraName>onboard_camera</cameraName> <robotNamespace>drone/%i</robotNamespace> </plugin> <pose>0.08 0 0 0 0</pose> <camera name='onboard_camera_'> <horizontal_fov>1.57</horizontal_fov> <image> <width>320</width> <height>240</height> </image> <clip> <near>0.01</near> <far>20</far> </clip> </camera> <always_on>1</always_on> <update_rate>10</update_rate> <visualize>0</visualize> </sensor> <visual name='brazo_NE'> <pose>0.0550 -0.0346 0 0 0 -1.0472</pose> <geometry> <box> <size>0.08 .01 .01</size> </box> </geometry> <material> <ambient>0.3 0.3 0.3 1</ambient> <diffuse>0.3 0.3 0.3 1</diffuse> </material> </visual> <collision name='brazo_NE'> <pose>0.0550 -0.0346 0 0 0 -1.0472</pose> <geometry> <box> <size>0.08 .01 .01</size> </box> </geometry> <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.2</mu2> <slip1>1.0</slip1> <slip2>1.0</slip2> </ode> </friction> </surface> </collision> <visual name='motor_NE'> <pose>0.0750 -0.0750 -0.01 0 0 0</pose> <geometry> <cylinder> <radius>.005</radius> <length>.04</length> </cylinder> </geometry> <material> <ambient>0 0 0 1</ambient> <!--color Red Green Blue Alpha [0-1] --> <diffuse>0 0 0 1</diffuse> </material> </visual> <collision name='motor_NE'> <pose>0.0750 -0.0750 -0.01 0 0 0</pose> <geometry> <cylinder> <radius>.005</radius> <length>.04</length> </cylinder> </geometry> <surface> <friction> <ode> <mu>100.0</mu> <mu2>50.0</mu2> <slip1>0.0</slip1> <slip2>0.0</slip2> </ode> </friction> </surface> </collision> <visual name='rotor_NE'> <pose>0.0750 -0.0750 0.01 0 0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>.005</length> </cylinder> </geometry> <material> <ambient>1 1 0 0.5</ambient> <!--color Red Green Blue Alpha [0-1] --> <diffuse>1 1 0 0.5</diffuse> </material> </visual> <collision name='rotor_NE'> <pose>0.0750 -0.0750 0.01 0 0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>.005</length> </cylinder> </geometry> <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.2</mu2> <slip1>1.0</slip1> <slip2>1.0</slip2> </ode> </friction> </surface> </collision> <visual name='brazo_NW'> <pose>0.0550 0.0346 0 0 0 1.0472</pose> <geometry> <box> <size>0.08 .01 .01</size> </box> </geometry> <material> <ambient>0.3 0.3 0.3 1</ambient> <diffuse>0.3 0.3 0.3 1</diffuse> </material> </visual> <collision name='brazo_NW'> <pose>0.0550 0.0346 0 0 0 1.0472</pose> <geometry> <box> <size>0.08 .01 .01</size> </box> </geometry> <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.2</mu2> <slip1>1.0</slip1> <slip2>1.0</slip2> </ode> </friction> </surface> </collision> <visual name='motor_NW'> <pose>0.0750 0.0750 -0.01 0 0 0</pose> <geometry> <cylinder> <radius>.005</radius> <length>.04</length> </cylinder> </geometry> <material> <ambient>0 0 0 1</ambient> <!--color Red Green Blue Alpha [0-1] --> <diffuse>0 0 0 1</diffuse> </material> </visual> <collision name='motor_NW'> <pose>0.0750 0.0750 -0.01 0 0 0</pose> <geometry> <cylinder> <radius>.005</radius> <length>.04</length> </cylinder> </geometry> <surface> <friction> <ode> <mu>100.0</mu> <mu2>50.0</mu2> <slip1>0.0</slip1> <slip2>0.0</slip2> </ode> </friction> </surface> </collision> <visual name='rotor_NW'> <pose>0.0750 0.0750 0.01 0 0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>.005</length> </cylinder> </geometry> <material> <ambient>1 1 0 0.5</ambient> <!--color Red Green Blue Alpha [0-1] --> <diffuse>1 1 0 0.5</diffuse> </material> </visual> <collision name='rotor_NW'> <pose>0.0750 0.0750 0.01 0 0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>.005</length> </cylinder> </geometry> <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.2</mu2> <slip1>1.0</slip1> <slip2>1.0</slip2> </ode> </friction> </surface> </collision> <visual name='brazo_SE'> <pose>-0.0550 -0.0346 0 0 0 1.0472</pose> <geometry> <box> <size>0.08 .01 .01</size> </box> </geometry> <material> <ambient>0.3 0.3 0.3 1</ambient> <diffuse>0.3 0.3 0.3 1</diffuse> </material> </visual> <collision name='brazo_SE'> <pose>-0.0550 -0.0346 0 0 0 1.0472</pose> <geometry> <box> <size>0.08 .01 .01</size> </box> </geometry> <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.2</mu2> <slip1>1.0</slip1> <slip2>1.0</slip2> </ode> </friction> </surface> </collision> <visual name='motor_SE'> <pose>-0.0750 -0.0750 -0.01 0 0 0</pose> <geometry> <cylinder> <radius>.005</radius> <length>.04</length> </cylinder> </geometry> <material> <ambient>0 0 0 1</ambient> <!--color Red Green Blue Alpha [0-1] --> <diffuse>0 0 0 1</diffuse> </material> </visual> <collision name='motor_SE'> <pose>-0.0750 -0.0750 -0.01 0 0 0</pose> <geometry> <cylinder> <radius>.005</radius> <length>.04</length> </cylinder> </geometry> <surface> <friction> <ode> <mu>100.0</mu> <mu2>50.0</mu2> <slip1>0.0</slip1> <slip2>0.0</slip2> </ode> </friction> </surface> </collision> <visual name='rotor_SE'> <pose>-0.0750 -0.0750 0.01 0 0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>.005</length> </cylinder> </geometry> <material> <ambient>1 0 1 0.5</ambient> <!--color Red Green Blue Alpha [0-1] --> <diffuse>1 0 1 0.5</diffuse> </material> </visual> <collision name='rotor_SE'> <pose>-0.0750 -0.0750 0.01 0 0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>.005</length> </cylinder> </geometry> <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.2</mu2> <slip1>1.0</slip1> <slip2>1.0</slip2> </ode> </friction> </surface> </collision> <visual name='brazo_SW'> <pose>-0.0550 0.0346 0 0 0 -1.0472</pose> <geometry> <box> <size>0.08 .01 .01</size> </box> </geometry> <material> <ambient>0.3 0.3 0.3 1</ambient> <diffuse>0.3 0.3 0.3 1</diffuse> </material> </visual> <collision name='brazo_SW'> <pose>-0.0550 0.0346 0 0 0 -1.0472</pose> <geometry> <box> <size>0.08 .01 .01</size> </box> </geometry> <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.2</mu2> <slip1>1.0</slip1> <slip2>1.0</slip2> </ode> </friction> </surface> </collision> <visual name='motor_SW'> <pose>-0.0750 0.0750 -0.01 0 0 0</pose> <geometry> <cylinder> <radius>.005</radius> <length>.04</length> </cylinder> </geometry> <material> <ambient>0 0 0 1</ambient> <!--color Red Green Blue Alpha [0-1] --> <diffuse>0 0 0 1</diffuse> </material> </visual> <collision name='motor_SW'> <pose>-0.0750 0.0750 -0.01 0 0 0</pose> <geometry> <cylinder> <radius>.005</radius> <length>.04</length> </cylinder> </geometry> <surface> <friction> <ode> <mu>100.0</mu> <mu2>50.0</mu2> <slip1>0.0</slip1> <slip2>0.0</slip2> </ode> </friction> </surface> </collision> <visual name='rotor_SW'> <pose>-0.0750 0.0750 0.01 0 0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>.005</length> </cylinder> </geometry> <material> <ambient>1 0 1 0.5</ambient> <!--color Red Green Blue Alpha [0-1] --> <diffuse>1 0 1 0.5</diffuse> </material> </visual> <collision name='rotor_SW'> <pose>-0.0750 0.0750 0.01 0 0 0</pose> <geometry> <cylinder> <radius>0.05</radius> <length>.005</length> </cylinder> </geometry> <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.2</mu2> <slip1>1.0</slip1> <slip2>1.0</slip2> </ode> </friction> </surface> </collision> <inertial> <mass>0.300</mass> <!-- <box><size>0.40 0.40 0.05</size></box> --> <inertia> <!-- http//en.wikipedia.org/wiki/List_of_moments_of_inertia --> <ixx>0.001022500</ixx> <!-- for a box ixx = 1/12 * mass * (y*y + z*z) --> <ixy>0.0</ixy> <!-- for a box ixy = 0 --> <ixz>0.0</ixz> <!-- for a box ixz = 0 --> <iyy>0.001022500</iyy> <!-- for a box iyy = 1/12 * mass * (x*x + z*z) --> <iyz>0.0</iyz> <!-- for a box iyz = 0 --> <izz>0.002000000</izz> <!-- for a box izz = 1/12 * mass * (x*x + y*y) --> </inertia> </inertial> </link> </model> </sdf>";

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
    drone_def = sprintf(drone_model, drone_id, drone_init_pos(1), drone_init_pos(2), drone_init_pos(3), drone_id, drone_id, drone_id);
    ros_msg.Data = drone_def;
    %Envio del mensaje por el topico
    send(inserter_publisher, ros_msg);
    pause(2);
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

function scheduleSimulation(simulation_scheduled, stop_simulation)
    IP = "192.168.2.111";

request = matlab.net.http.RequestMessage;
uri_deploy = matlab.net.URI(sprintf('http://%s:3000/deploy/dronechallenge.launch', IP));
uri_destroy = matlab.net.URI(sprintf('http://%s:3000/destroy', IP));

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
            pause(2);
        else
            disp('Stopping simulation.');
            simulation_scheduled = 0;
            response = send(request, uri_destroy);
            disp(response.Body.Data);
        end
    end
end
end
