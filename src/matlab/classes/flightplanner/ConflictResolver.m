classdef ConflictResolver < handle
    %CONFLICTRESOLVER Class to resolve conflicts between two or more flight plans
    
    properties
        conf_dist;
    end
    
    methods
        function obj = ConflictResolver(conf_dist)
            %CONFLICTRESOLVER Construct an instance of this class
            %   Detailed explanation goes here
            obj.conf_dist = conf_dist;
        end
    
        function fps = simplerResolve(obj, fpc)
            %SIMPLERRESOLVE Reslve conflicts between two or more flight plans moving waypoints
            it = 1;

            while true
                conflicts = fps.detectConflicts(obj.conf_dist,0.5);
                fprintf("Conflict at it %d: %d \n", it, size(conflicts,1));
                %disp("-----")
                %fps.routesFigure();

                %Check if there are any conflicts
                if isempty(conflicts)
                    return
                end

                %Unpack conflict info
                id1 = conflicts(1,1);
                id2 = conflicts(1,2);
                dist = obj.conf_dist - conflicts(1,3) + 0.1;
                t = conflicts(1,4);

                %Get flightplan that produce the conflict
                fp1 = fps.getFlightPlanById(id1);
                fp2 = fps.getFlightPlanById(id2);

                %Check if fps are valid
                if isempty(fp1) || isempty(fp2)
                    disp("Error: Flight plan not found. Check IDs and try again.");
                    return
                end
            
                %Get the waypoint that produce the conflict
                wp1 = fp1.getMostCloseWayByTime(t);
                wp2 = fp2.getMostCloseWayByTime(t);

                %Compute the vectors between the two waypoints
                v1 = [wp1.x, wp1.y ,wp1.z] - [wp2.x, wp2.y ,wp2.z];
                v2 = -v1;

                %Unit vectors
                v1_u = v1/norm(v1);
                v2_u = v2/norm(v2);

                %Distance to move each waypoint
                d1 = v1_u*(dist/2);
                d2 = v2_u*(dist/2);

                %Move waypoints
                wp1.changeLocation(wp1.x + d1(1), wp1.y + d1(2), wp1.z + d1(3));
                wp2.changeLocation(wp2.x + d2(1), wp2.y + d2(2), wp2.z + d2(3));
                it = it+1;
            end
        end
    end
end

