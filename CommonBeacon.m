clc
clear
beacon_pos = [0 0];
num_robots = 4;
t = 10;
dt = 0.1;
ellipse = zeros(num_robots,2);
ellipse = [1 2;
           2 1;
           4 2;
           2 4];
omega = [2*pi/4;
        2*pi/4;
        2*pi/8;
        2*pi/8];
Q = [0.01 0;0 0.02];
R = 0.01;
robpos_true = zeros(num_robots,2,t/dt);
robpos_est = movement(ellipse,omega,num_robots,0);
P_rob = zeros(2,2,num_robots);
P_rec = zeros(num_robots,4,t/dt);
robpos_old = movement(ellipse,omega,num_robots,0);
robpos_rec = zeros(num_robots,2,t/dt);
for i = 1:t/dt
    robpos_true(:,:,i) = movement(ellipse,omega,num_robots,i*dt);
    for j = 1:num_robots
        odom = robpos_true(j,:,i) - robpos_old(j,:) +  mvnrnd([0;0],Q);
        robpos_est(j,:) = robpos_est(j,:) + odom;
        P_rob(:,:,j) = P_rob(:,:,j) + Q;
        meas(j) = sqrt((robpos_true(j,1,i) - beacon_pos(1))^2 + (robpos_true(j,2,i) - beacon_pos(2))^2);
    end
    for j = 1:num_robots
        pred = sqrt((robpos_est(j,1) - beacon_pos(1))^2 + (robpos_est(j,2) - beacon_pos(2))^2);
        innov = meas(j) - pred;
        H = [(robpos_est(j,1) - beacon_pos(1)) (robpos_est(j,2) - beacon_pos(2))]/pred;
        S = H*P_rob(:,:,j)*H' + R;
        K = P_rob(:,:,j)*H'*inv(S);
        robpos_est(j,:) = robpos_est(j,:) + (K*innov)';
        P_rob(:,:,j) = (eye(2,2) - K*H)*P_rob(:,:,j);
        P_rec(j,1:2,i) = P_rob(1,:,j);
        P_rec(j,3:4,i) = P_rob(2,:,j);
    end
    robpos_rec(:,:,i) = robpos_est(:,:);
end

figure(1);

subplot(2,1,1);

plot(1:t/dt,squeeze(robpos_rec(1,1,:)),'b');

hold on;

plot(1:t/dt,3*sqrt(squeeze(P_rec(1,1,:))),'r');

plot(1:t/dt,-3*sqrt(squeeze(P_rec(1,1,:))),'r');

title('x');

subplot(2,1,2);

plot(1:t/dt,squeeze(robpos_rec(1,2,:)),'b');

hold on;

plot(1:t/dt,3*sqrt(squeeze(P_rec(1,4,:))),'r');

plot(1:t/dt,-3*sqrt(squeeze(P_rec(1,4,:))),'r');

title('y');



function robpos = movement(ellipse,omega,num_robots,t)
    robpos = zeros(num_robots,2);
    for i = 1:num_robots
        robpos(i,1) = ellipse(i,1)*cos(t*omega(i));
        robpos(i,2) = ellipse(i,2)*sin(t*omega(i));
    end
end




function DisplayINIT(WorldSize)
figure(1); 
subplot(2,2,1); hold on; grid off; axis equal;
set(gcf,'doublebuffer','on');
hLine = line([0,0],[0,0]);
set(hLine,'linestyle',':');
axis([ -WorldSize/2 WorldSize/2 -WorldSize/2 WorldSize/2]);
% axis equal
xlabel(' Initial Conditions and beacons at read +');
drawnow
end

function Animate(num_robots,time_count,dt,x_true,xr,Pr,trust)
figure(1); clf;
WorldSize = 20;
axis([ -WorldSize/2 WorldSize/2 -WorldSize/2 WorldSize/2]);
t = time_count*dt;
for i = 1:num_robots
    hold on;
    DrawRobot(x_true(:,i),xr(:,i),expand(Pr(i,:)'),i);
end
subplot(2,2,2);
plot(dt:dt:t,trust(1:time_count,1,2));
hold on;
plot(dt:dt:t,trust(1:time_count,1,3));
ylim([0 1]);
subplot(2,2,3);
plot(dt:dt:t,trust(1:time_count,2,1));
hold on;
plot(dt:dt:t,trust(1:time_count,2,3));
ylim([0 1]);
subplot(2,2,4);
plot(dt:dt:t,trust(1:time_count,3,1));
hold on;
plot(dt:dt:t,trust(1:time_count,3,2));
ylim([0 1]);
%pause(0.01);
end