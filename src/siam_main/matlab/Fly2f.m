
Fly2(1,1,1);

function d3D = Fly2(X,Y,Z)
    fprintf("Volando a %f, %f, %f \n", X, Y ,Z);
    
    % target distance decomposed into earth axes
    %vector between the commanded position and the drone position
    dxe = X - 0;
    dye = Y - 0;
    dze = Z - 0;
    
    % target distance
    % vector3D modulus
    d3D = norm([dxe;dye;dze]);
    
    % target distance on the plane
    %vector2D modulus
    d2D = norm([dxe;dye]);
    
    % angle between drone heading and target
    %As commanded velocity is relative to drone heading, 
    %difference between heading and target must be computed in order to compute
    %the correct drone displacement
    bearing = atan2d(dye, dxe) - 0;
    
    % target distance decomposed into drone body axes
    %distance transformed to drone taking into account drone heading
    dx = d2D * cosd(bearing);
    dy = d2D * sind(bearing);
    dz = dze;
    
    % velocity obtained from distance
    % speed softened (50%)
    vel = [dx;dy;dz]/2;
    % speed normalization
    if norm(vel) > 1 
        vel = vel / norm(vel);
    end
    
    fprintf("Volando a %f, %f, %f. Distance: %f \n", X, Y, Z, d3D);
    
    %Speeed commanded
    cmd.velX = vel(1);
    cmd.velY = vel(2);
    cmd.velZ = vel(3);
end