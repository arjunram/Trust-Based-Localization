clc
clear

x1 = 100;
x2 = -50;
X = [x1;x2];
t1 = -1.2;
t2 = 1.1;
T = [t1;t2];
y0 = [X;T];

tfinal = 0.001;

options = odeset('Events',@(t,y) event_function(t,y,y0),'RelTol',1e-4,'AbsTol',1e-6);
[t,y] = ode45(@(t,y) kin(t,y),[0 tfinal],y0,options);

plot(t,y(:,1),'b');
hold on;
plot(t,y(:,2),'r');

figure
plot(t,y(:,3),'b');
hold on
plot(t,y(:,4),'r');


function diff = kin(t,state)
diff = zeros(4,1);
x1 = state(1);
x2 = state(2);
t1 = state(3);
t2 = state(4);
diff(1) = t1*(x2 - x1);
diff(2) = t2*(x1 - x2);
diff(3) = t2*(x1-x2)^2;
diff(4) = t1*(x1-x2)^2;
end

function [value,isterminal,direction] = event_function(t,y,y0)
if norm(y(1)-y(2))<0.0001
    value = 0; % when value = 0, an event is triggered
else
    value = 1;
end
isterminal = 0; % terminate after the first event
direction = 0;  % get all the zeros
end