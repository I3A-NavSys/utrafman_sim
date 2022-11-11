for x = 1:numDrones    
    dron = drone(x);
    pos = zeros(1,size(dron.locs,2));
    rot = zeros(1,size(dron.locs,2));
    t = zeros(1,size(dron.locs,2));
    for i=1:size(dron.locs,2)
        pos(i) = dron.locs(i).Pose.Orientation.Z;
        rot(i) = dron.locs(i).Velocity.Angular.Z;
        t(i) = dron.locs(i).Time.Sec;
    end
    posi = cumtrapz(rot(:));
    
    figure(x);
    plot(t,rot);
    hold on;
    grid on;
    ylim([-pi,pi])
    plot(t,pos);
    % plot(t,posi);
    hold off;
end