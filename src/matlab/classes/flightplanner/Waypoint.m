classdef Waypoint < handle
    %WAYPOINT This class represent a flight plan's waypoint
    %   Detailed explanation goes here
    
    properties

        % instante de tiempo
        % establecido desde un instante inicial prefijado
        t    {mustBeNumeric}    % segundos a futuro.
    
        % posicion en espacio 3D
        % establecida sobre un eje de referencia prefijado en el escenario
        x    {mustBeNumeric}    % metros hacia el este
        y    {mustBeNumeric}    % metros hacia el norte
        z    {mustBeNumeric}    % metros hacia arriba
    
        % velocidad en espacio 3D
        % establecida sobre un eje de referencia prefijado en el escenario
        vx   {mustBeNumeric}    % metros/s hacia el este
        vy   {mustBeNumeric}    % metros/s hacia el norte
        vz   {mustBeNumeric}    % metros/s hacia arriba

        % Waypoint de obligado tránsito (en un plan de vuelo)
        % valor lógico (verdadero/falso)
        mandatory logical;
    end
    
    methods

        function obj = Waypoint()
            %WAYPOINT Constructor for the class
            obj.t  = 0;
            obj.x  = 0;
            obj.y  = 0;
            obj.z  = 0;
            obj.vx = 0;
            obj.vy = 0;
            obj.vz = 0;
            obj.mandatory = true;
        end


        function checkWaypoint(obj,wp)
            if ~isa(wp,'Waypoint')
               error('Error. \nValue must be a Waypoint object, not a %s.',class(wp))
            end
        end


        function p = position(obj)
            %POSITION Get the position of the waypoint
            p = [ obj.x  obj.y  obj.z ];
        end
       

        function setPosition(obj,p)
            %SET_POSITION Set the position of the waypoint
            obj.x = p(1);
            obj.y = p(2);
            obj.z = p(3);
        end
  

        function v = velocity(obj)
            %POSITION Get the velocity of the waypoint
            v = [ obj.vx  obj.vy  obj.vz ];
        end
       

        function setVelocity(obj,v)
            %SET_VELOCITY Set the velocity of the waypoint
            obj.vx = v(1);
            obj.vy = v(2);
            obj.vz = v(3);
        end


        function time = timeTo(a,b)
            %TIMETO Get the time elapsed from this waypoint to another given
            a.checkWaypoint(b);
            time = b.t - a.t;
        end


        function dist = distanceTo(a,b)
            %DISTANCE Get the distance between two waypoints
            a.checkWaypoint(b);
            dist = norm(a.position - b.position);
        end


        function dir = directionTo(a,b)
            %DISTANCE Get a direction vector from one waypoint to another            a.checkWaypoint(b);
            a.checkWaypoint(b);
            dist = a.distanceTo(b);
            if dist == 0
                dir = [0,0,0];
            else
                dir = (b.position - a.position) / dist;
            end
        end


        function vel = uniformVelocityTo(a,b)
            %VELOCITY Get the uniform velocity between two waypoints
            a.checkWaypoint(b);

            dist = a.distanceTo(b);
            time = a.timeTo(b);

            if time == 0
                vel = 0;
            else
                vel = dist / time;
            end
        end
    
    
        function wp3 = interpolationTP(wp1,wp2,t)
            %INTERPOLATIONTP 
            % dados dos waypoints, 
            % genera un tercer waypoint interpolando posiciones a un tiempo dado.
            % Equivale a realizar un movimiento rectilíneo y uniforme.
            wp1.checkWaypoint(wp2);
                    
            wp3 = Waypoint;
            wp3.t = t;
            s13 = wp1.directionTo(wp2) * wp1.uniformVelocityTo(wp2) * wp1.timeTo(wp3);
            wp3.setPosition(wp1.position + s13);

        end


        function wp3 = interpolationTPV(wp1,wp2,t)
            % PENDIENTE DE CHEQUEAR
            %INTERPOLATIONTPV
            % dados dos waypoints, 
            % genera un tercer waypoint interpolando posiciones y velocidades a un tiempo dado.
            wp1.checkWaypoint(wp2);

            wp3 = Waypoint;
            wp3.t = t;
        
            % s12 = wp1.distanceTo(wp2);
            s12 = wp2.position - wp1.position;
            if norm(s12)==0
                wp3.setPosition(wp1.position);
                return
            end
            
            t12 = wp1.timeTo(wp2);
            t13 = wp1.timeTo(wp3);
            v1  = wp1.velocity;
            v2  = wp2.velocity;
            
            A = [ t12^2/2   t12^3/6 ;
                  t12       t12^2/2 ];

            B = [ s12-v1*t12 ;
                  v2-v1      ];


            if rank(A) == 2
                X = A\B;
                a = X(1,:);
                j = X(2,:);
            else
                error('Error. Interpolación TPV sin solución')
            end  
        
            s13 = v1*t13 + 1/2 *a*t13^2 + 1/6 *j*t13^3;
            wp3.setPosition( wp1.position + s13 );
        
        end
    
    end
end