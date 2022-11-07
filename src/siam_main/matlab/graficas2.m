figure(1);

t = 1:0.1:10;
x = sin(t);
y = cos(t);
z = tan(t);

xref = y;
yref = x;
zref = -z;

waypoints = 2 .* rand([4,3]) - 1;

%3D viewer
subplot(2,2,[1 3]);
plot3(x,y,z);
grid on;
hold on;
plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),'or');
title("Route 3D view")
xlabel("pos X");
ylabel("pos Y");
zlabel("pos Z");
hold off;


%X,Y, Z pos
var_names = ["Simulated", "Reference"];
tab = table([x',xref'], [y', yref'], [z', zref'],'VariableNames',{'X','Y','Z'});
subplot(2,2,2);
stackedplot(tab);
title("Positions through time");
xlabel('Simulated time');
grid on;

%X,Y, Z accel
var_names = ["Simulated", "Reference"];
tab = table([diff(x)',diff(xref)'], [diff(y)', diff(yref)'], [diff(z)', diff(zref)'],'VariableNames',{'X','Y','Z'});
subplot(2,2,4);
stackedplot(tab);
title("Velocities through time");
xlabel('Simulated time');
grid on;

drawnow;
