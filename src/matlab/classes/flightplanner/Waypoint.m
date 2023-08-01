classdef Waypoint %< handle
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
    
        % velocidad de desplazamiento
        % expresada en modulo, sin direccion en espacio 3D
        v    {mustBeNumeric}    % metros/s

        % Waypoint de obligado tránsito (en un plan de vuelo)
        % valor lógico (verdadero/falso)
        mandatory logical;
    end
    
    methods

        function obj = Waypoint(t,x,y,z,v,m)
            %WAYPOINT Constructor for the class
            obj.t = t;
            obj.x = x;
            obj.y = y;
            obj.z = z;
            obj.v = v;
            obj.mandatory = m;
        end


        function obj = setPosition(obj,p)
            %SET_POSITION Set the position of the waypoint
            obj.x = p(1);
            obj.y = p(2);
            obj.z = p(3);
        end


        function p = position(obj)
            %POSITION Get the position of the waypoint
            p = [ obj.x  obj.y  obj.z ];
        end


        function time = timeTo(a,b)
            %Check if A and B are Waypoints objects
            if ~isa(a,'Waypoint') || ~isa(b,'Waypoint')
                error('A and B must be Waypoint objects');
            end

            %TIME Calculate the time between two waypoints
            time = b.t - a.t;
        end


        function dist = distanceTo(a,b)
            %Check if A and B are Waypoints objects
            if ~isa(a,'Waypoint') || ~isa(b,'Waypoint')
                error('A and B must be Waypoint objects');
            end

            %DISTANCE Calculate the distance between two waypoints
            dist = norm(a.position - b.position);
        end


        function vel = velocityTo(a,b)
            %VELOCITY Calculate the velocity between two waypoints

            %Check if A and B are Waypoints objects
            if ~isa(a,'Waypoint') || ~isa(b,'Waypoint')
                error('A and B must be Waypoint objects');
            end

            dist = a.distanceTo(b);
            time = a.timeTo(b);

            if time == 0
                vel = 0;
            else
                vel = dist / time;
            end
        end
    
    
        function wp3 = interpolation4D(obj,wp2,t)
            wp3   = Waypoint;
            wp3.t = t;
        
            p1  = obj.position;
            p2  = wp2.position;
            t12 = obj.timeTo(wp2);
            t13 = obj.timeTo(wp3);

            if t12==0
                wp3 = setPosition(p1);
            else
                wp3.setPosition(p1 + (p2 - p1) * t13 / t12);
            end

        end


        function wp3 = interpolation5D(obj,wp2,t)
            wp3 = Waypoint;
        
            s12 = obj.distanceTo(wp2);
            if s12==0
                wp3 = obj;
                return
            end
            
            t12 = wp2.t - obj.t;
            t13 =    t - obj.t;
            v1  = obj.v;
            v2  = wp2.v;
            
            A = [ t12^2/2   t12^3/6 ;
                  t12       t12^2/2 ];
            B = [ s12-v1*t12 ;
                  v2-v1    ];
            if rank(A) == 2
                X = A\B;
                a = X(1);
                j = X(2);
            else
                disp('NO existe solución')
                return
            end  
        
            s13 = v1*t13 + 1/2 *a*t13^2 + 1/6 *j*t13^3;
        
            wp3.x = obj.x + (wp2.x-obj.x) * s13 / s12;
            wp3.y = obj.y + (wp2.y-obj.y) * s13 / s12;
            wp3.z = obj.z + (wp2.z-obj.z) * s13 / s12;
            wp3.t = obj.t + (wp2.t-obj.t) * s13 / s12;
            
        end
    
    end
end