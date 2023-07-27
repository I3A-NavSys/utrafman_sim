classdef Waypoint < handle
    %WAYPOINT This class represent a flight plan's waypoint
    %   Detailed explanation goes here
    
    properties
        id int32;
        x double;
        y double;
        z double;
        t double;
        r double;
        mandaroty logical;
    end
    
    methods
        function obj = Waypoint(x,y,z,t,r,m)
            %WAYPOINT Constructor for the class
            obj.x = x;
            obj.y = y;
            obj.z = z;
            obj.t = t;
            obj.r = r;
            obj.mandaroty = m;
        end

        function isw = isWaypoint(obj)
            %ISWAYPOINT Check if the object is a Waypoint
            if ~isa(obj,'Waypoint')
                isw = false;
                return;
            end
        end
        
        function obj = changeLocation(obj,x,y,z)
            %CHANGE_LOCATION Change the location of the waypoint
            obj.x = x;
            obj.y = y;
            obj.z = z;
        end

        function obj = changeTime(obj,t)
            %CHANGE_TIME Change the time of the waypoint
            obj.t = t;
        end

        function obj = changeRadius(obj,r)
            %CHANGE_RADIUS Change the radius of the waypoint
            obj.r = r;
        end

        function obj = changeMandatory(obj,m)
            %CHANGE_MANDATORY Change the mandatory of the waypoint
            obj.mandaroty = m;
        end

        function equal = isequal(a, b)
            %Check if A and B are Waypoints objects
            if ~isa(a,'Waypoint') || ~isa(b,'Waypoint')
                error('A and B must be Waypoint objects');
            end

            %ISEQUAL Check if two waypoints are equal
            equal = (a.id == b.id);
        end

        function dist = distanceWithWaypoint(a, b)
            %Check if A and B are Waypoints objects
            if ~isa(a,'Waypoint') || ~isa(b,'Waypoint')
                error('A and B must be Waypoint objects');
            end

            %DISTANCE Calculate the distance between two waypoints
            dist = norm([a.x, a.y, a.z] - [b.x, b.y, b.z]);
        end

        function vel = velocityWithWaypoint(a, b)
            %Check if A and B are Waypoints objects
            if ~isa(a,'Waypoint') || ~isa(b,'Waypoint')
                error('A and B must be Waypoint objects');
            end

            %VELOCITY Calculate the velocity between two waypoints
            vel = a.distanceWithWaypoint(b) / (b.t - a.t);
        end
    end
end