classdef GazeboConnector < handle
    
properties

    ROS_MASTER_IP string
    ROSnode             % ROS Node handler
    ROSsub_clock;       % Subscriber to Gazebo clock
    ROScli_play;        % Service Client to start Gazebo simulation
    ROScli_pause;       % Service Client to pause Gazebo simulation
    ROScli_reset;       % Service Client to reset Gazebo simulation

end %properties

methods


    function obj = GazeboConnector(ROS_MASTER_IP)

    %Connection with ROS Master
    obj.ROS_MASTER_IP = ROS_MASTER_IP;
    try
        rosinit(ROS_MASTER_IP);
    catch
        disp("Restarting connection...");
        rosshutdown;
        rosinit(ROS_MASTER_IP);
    end

    % ROS node
    obj.ROSnode = ros.Node("/utm/Gazebo_connector",obj.ROS_MASTER_IP,11311);


    % Gazebo connections
    obj.ROSsub_clock = ros.Subscriber(obj.ROSnode, "/clock");
    obj.ROScli_play  = ros.ServiceClient(obj.ROSnode, "/gazebo/unpause_physics");
    obj.ROScli_pause = ros.ServiceClient(obj.ROSnode, "/gazebo/pause_physics");
    obj.ROScli_reset = ros.ServiceClient(obj.ROSnode, "/gazebo/reset_simulation");

    disp("Connected to Gazebo simulator.");
end


% Updating Gazebo simulation time
function sec = timeSeconds(obj)
    [msg, status] = receive(obj.ROSsub_clock, 1);
    if status 
        % message was received
        sec = msg.Clock_.Sec;
    else
        sec = -1;
    end
end


function play(obj)

    msg = obj.ROScli_play.rosmessage;
    if(isServerAvailable(obj.ROScli_play))
        [connectionStatus,connectionStatustext] = waitForServer(obj.ROScli_play);
        if connectionStatus
            call(obj.ROScli_play,msg,"Timeout",5);
        else
            error("Error playing Gazebo simulation");
        end
    else
        error("Gazebo server is not available");
    end

end


function pause(obj)

    msg = obj.ROScli_pause.rosmessage;
    if(isServerAvailable(obj.ROScli_pause))
        [connectionStatus,connectionStatustext] = waitForServer(obj.ROScli_pause);
        if connectionStatus
            call(obj.ROScli_pause,msg,"Timeout",5);
        else
            error("Error pausing Gazebo simulation");
        end
    else
        error("Gazebo server is not available");
    end

end


function reset(obj)
    
    msg = ROScli_reset.rosmessage;
    if(isServerAvailable(ROScli_reset))
        [connectionStatus,connectionStatustext] = waitForServer(ROScli_reset);
        if connectionStatus
            call(ROScli_reset,msg,"Timeout",5);
        else
            error("Error reseting Gazebo simulation");
        end
    else
        error("Gazebo server is not available");
    end

end


function disconnect(obj)

    % ROS disconnection
    rosshutdown;
    disp("Disconnected of Gazebo simulator.");

end




% %To remove all models from simulation
% function obj = removeAllModelsFromSimulation(obj)
% 
%     client = rossvcclient('/godservice/remove_model');
% 
%     for i = obj.S_Registry.uavs
%         drone = i;
%         msg = rosmessage(client);
%         msg.UavId = drone.Id;
%         call(client, msg, 'Timeout', 10);
%     end
% end


% %To set the automatically finish simulation time
% %Maintains the greatest time passed on all method calls
% function setFinishSimulationTime(obj, finish_time)
%     if (obj.finish_simulation_time == inf) || (finish_time > obj.finish_simulation_time)
%         obj.finish_simulation_time = finish_time;
%     end
%     if isempty(obj.finish_simulation_timer)
%         obj.finish_simulation_timer = timer("TimerFcn", @obj.checkIfTimeToFinishSimulation,"ExecutionMode","fixedRate", "Period",5);
%         start(obj.finish_simulation_timer);
%     end
% end


% %Check if is time to finish the simulation
% function obj = checkIfTimeToFinishSimulation(obj, timer, time)
%     if obj.Gclock > obj.finish_simulation_time
%         obj.finishSimulation();
%     end
% end



end %methods
end %classdef

