%UTM Airspace class represent and entire airspace with all the entities, services and information needed to manage the airspace
classdef Gazebo < handle
    
properties

    %ROS network info
    ROS_MASTER_IP   string

    %Gazebo Clock
%    Gclock = -1;            %Gazebo clock value (seconds)
    GClock_sub;             %Subscriber to Gazebo clock
%    GClock_timer;           %Timer to update Gazebo clock

end %properties

methods


function obj = Gazebo(ROS_MASTER_IP)

    %Clean all (previous) timers
    timers = timerfind;
    if ~isempty(timers)
        %disp("Deleting timers.");
        stop(timers);
        delete(timers);
    end

    %Connection with ROS Master
    obj.ROS_MASTER_IP = ROS_MASTER_IP;
    try
        rosinit(ROS_MASTER_IP);
    catch
        disp("Restarting connection...");
        rosshutdown;
        rosinit(ROS_MASTER_IP);
    end
    disp("Connected to Gazebo simulator.");

    %Suscribe to Gazebo clock
    obj.GClock_sub = rossubscriber("/clock");

    % %Configure timer to update Gazebo clock every 1sec
    % %(it is used because clock topic is published with a very high frequency)
    % obj.GClock_timer = timer(...
    %     'ExecutionMode','fixedRate',...
    %     'Period',1,...
    %     'TimerFcn',@obj.updateGclock);
    % start(obj.GClock_timer);
    % disp("Clock synchronized.");

end

%Updating Gazebo simulation time
function sec = timeSeconds(obj)
    [msg, status] = receive(obj.GClock_sub, 0.1);
    if status 
        % message was received
        sec = msg.Clock_.Sec;
    else
        sec = -1;
    end
end


%Updating Gazebo simulation time
function updateGclock(obj,~,~)
%    disp('clock')
    [msg, status] = receive(obj.GClock_sub, 0.1);
    if status %a message was received
        obj.Gclock = msg.Clock_.Sec;
    end
end


function play(obj)

    client = rossvcclient('/gazebo/unpause_physics');
    msg = client.rosmessage;
    if isServerAvailable(client)
        call(client,msg,"Timeout",1);
    % else
    %     error("Error playing Gazebo simulation");
    end

end


function pause(obj)

    client = rossvcclient('/gazebo/pause_physics');
    msg = client.rosmessage;
    if isServerAvailable(client)
        call(client,msg,"Timeout",1);
    % else
    %     error("Error pausing Gazebo simulation");
    end

end


function reset(obj)
    
    client = rossvcclient('/gazebo/reset_simulation');
    msg = client.rosmessage;
    if isServerAvailable(client)
        call(client,msg,"Timeout",1);
    % else
    %     error("Error reseting Gazebo simulation");
    end
    obj.Gclock = 0;

end


function disconnect(obj)

    %Clean clock timers
    stop(timerfind);
    delete(timerfind);

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

