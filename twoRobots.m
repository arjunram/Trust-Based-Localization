clc
clear

x1 = 100;
x2 = -50;
X = [x1;x2];
Q = 0.1*eye(2,2);%[0.1 0;0 0.9];%0.1*eye(2,2);
R = 0.2*eye(2,2);%[0.01 0;0 1.5];%0.2*eye(2,2);
xhat1 = 102;
xhat2 = -55;
Xhat = [xhat1;xhat2];
P = eye(2,2);
t1 = 1;
t2 = 1;
T = [t1;t2];
y0 = [X;Xhat;T;P(1,1);P(1,2);P(2,1);P(2,2)];

tfinal = 5;

options = odeset('Events',@(t,y) event_function(t,y,y0),'RelTol',1e-4,'AbsTol',1e-6);
[t,y] = ode45(@(t,y) kin(t,y,Q,R),[0 tfinal],y0,options);

plot(t,y(:,1));
hold on;
plot(t,y(:,2),'r');
figure
plot(t,y(:,5));
hold on
plot(t,y(:,6));
figure;
hold on;
plot(t,y(:,3)-y(:,1));
%plot(t,y(:,7),'r');
%plot(t,-y(:,7),'r');

figure;
hold on;
plot(t,y(:,4)-y(:,2));
%plot(t,y(:,10),'r');
%plot(t,-y(:,10),'r');


function diff = kin(t,state,Q,R)
X = state(1:2);
Xhat = state(3:4);
t1 = state(5);
t2 = state(6);
P = [state(7) state(8);state(9) state(10)];
w = mvnrnd([0; 0],Q);
v = mvnrnd([0; 0],R);
dist = (X(1)-X(2)); 
disthat = (Xhat(1) - Xhat(2));
z = [dist + v(1);-dist+v(2)];
zhat = [disthat;-disthat];
F = [-t1 t1;t2 -t2];%[-1 1;1 -1];
H = [1 -1;-1 1];
K = [t1 0;0 t2]*P*H'/R;
diff = zeros(10,1);
diff(1) = t1*(Xhat(2) - Xhat(1))+w(1);
diff(2) = t2*(Xhat(1) - Xhat(2))+w(2);
diff(3:4) = F*Xhat + K*(z-zhat);
diff(6) = (Xhat(1) - Xhat(2))*diff(4);
diff(5) = (Xhat(2) - Xhat(1))*diff(3);
Pdot = F*P + P*F' + Q - K*R*K'; 
diff(7:10) = [Pdot(1,1);Pdot(1,2);Pdot(2,1);Pdot(2,2)];
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