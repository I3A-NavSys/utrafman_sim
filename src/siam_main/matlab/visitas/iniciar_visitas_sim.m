%ROS
rosMasterIp = "192.168.1.131";
rosMasterPort = 11311;

%Modelos
userDroneModel = fileread('../../models/dronechallenge_models/drone/model_template_2_visitas.sdf');
droneModel = fileread('../../models/dronechallenge_models/drone/model_template_1.sdf');

%Conexion con el master de ROS
try
    rosinit(rosMasterIp, rosMasterPort);
catch
    disp("ROS ya está iniciado. Reiniciando la conexión ...");
    rosshutdown;
    rosinit(rosMasterIp, rosMasterPort);
end

%Creacion del suscriptor para insertar los drones
[rosInserterPub, rosInserterMsg] = rospublisher("/god/insert");

for id=1:5
    initPos = [0 -0.6*id+1.8 1];
    %Definimos que SDF debe usarse y generamos el modelo del drone
    if id ~= 3
    	droneSDF = sprintf(userDroneModel, id, initPos(1), initPos(2), initPos(3), id, id, id);
    else
    	droneSDF = sprintf(droneModel, id, initPos(1), initPos(2), initPos(3), id, id, id);
    end
    
    %Generamos el mensaje y lo enviamos por el topico
    rosInserterMsg.Data = droneSDF;
    send(rosInserterPub, rosInserterMsg);
    pause(.5);
end
