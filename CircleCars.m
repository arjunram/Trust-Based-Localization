clc
clear

tfinal = 20;
dt = 0.01;
lambda = 0.3;
W1 = 1;
W2 = 1.2;
W3 = 0.1;
sf = 1;
num_robots = 3;
trust = zeros(3);
comm_range = 3;
odom_mu = [0 0 0];
odom_sigma = [0.01 0 0; 0 0.02 0; 0 0 0.1];
odom_noise_flag = 0;
rang_mu = 0;
rang_sigma = 0.2;
rang_noise_flag = 0;
Pinit = [0.1; 0; 0; 0.2; 0; 0.05];
DisplayINIT(10);

for i = 1:num_robots
    ref = ref_traj(0,i);
    x_true(1,i) = ref(1);
    y_true(1,i) = ref(2);
    theta_true(1,i) = ref(3);
    x(1,i) = x_true(1,i);
    y(1,i) = y_true(1,i);
    theta(1,i) = theta_true(1,i);
    P_own(1,i,:) = Pinit;
    for j = i+1:num_robots
        P_nei(1,i,j,:) = Pinit;
        P_nei(1,j,i,:) = Pinit;
        trust(i,j) = 0.5;
        trust(j,i) = 0.5;
        trustRec(1,i,j) = trust(i,j);
        trustRec(1,j,i) = trust(j,i);
    end
end

%figure(1);
hold on;
time_count = 2;
for t = dt:dt:tfinal
    %Update robot locations and covariance
    for i = 1:num_robots
        ref = ref_traj(t,i);
        x_true(time_count,i) = ref(1);
        y_true(time_count,i) = ref(2);
        theta_true(time_count,i) = ref(3);
        noise = odom_noise_flag*mvnrnd(odom_mu,odom_sigma,1);
        odom = odom_meas(t,t-dt,i);
        x(time_count,i) = x(time_count-1,i) + odom(1) + noise(1);
        y(time_count,i) = y(time_count-1,i) + odom(2) + noise(2);
        theta(time_count,i) = theta(time_count-1,i) + odom(3) + noise(3);
        P_own(time_count,i,:) = shrink(expand(P_own(time_count -1,i,:)) + odom_sigma);
    end
    
    range = zeros(num_robots);
    %Perform ranging
    for i = 1:num_robots
        for j = i+1:num_robots
            range(i,j) = sqrt((x_true(i)-x_true(j))^2 + (y_true(i)-y_true(j))^2) + rang_noise_flag*normrnd(rang_mu,rang_sigma);
            range(j,i) = range(i,j);
        end
    end
    
    %trust calc
    for i = 1:num_robots
        for j = 1:num_robots
            if (i==j) 
                continue;
            end
            
            dist = sqrt((x(time_count,i)-x(time_count,j))^2 + (y(time_count,i)-y(time_count,j))^2);
            dist_err = range(i,j) - dist;
            if(dist<comm_range)
                P_nei(time_count,i,j,:) = P_own(time_count,j,:);
                trustdot = -lambda*trust(i,j) + W1*(1-trust(i,j))*scale_func(sf*vecnorm(squeeze(P_nei(time_count-1,i,j,:)-P_nei(time_count,i,j,:))) + W2*dist_err + W3*vecnorm(squeeze(P_nei(time_count,i,j,:))));
            else
                P_nei(time_count,i,j,:) = P_nei(time_count-1,i,j,:);
                trustdot = 0;
            end
            trust(i,j) = trust(i,j) + dt*trustdot;
            trustRec(time_count,i,j) = trust(i,j);

        end
    end
    Animate(num_robots,time_count,dt,[x_true(time_count,:);y_true(time_count,:);theta_true(time_count,:)],[x(time_count,:);y(time_count,:);theta(time_count,:)],squeeze(P_own(time_count,:,:)),trustRec);
    time_count = time_count +1;

end

function scale = scale_func(val)
scale = 1/(exp(val));
end

function odom = odom_meas(t_new,t_old,i)
ref_new = ref_traj(t_new,i);
ref_old = ref_traj(t_old,i);
odom = ref_new - ref_old;
end

function ref = ref_traj(t,i)
rad = i;
omega = 2*i*pi*0.1;
ref(1) = rad*cos(omega*t);
ref(2) = rad*sin(omega*t);
ref(3) = omega*t + pi/2;
if(ref(3)>2*pi) ref(3) = ref(3)-2*pi; end
if(ref(3)<0) ref(3) = ref(3)+2*pi; end
end

function P_vec = shrink(P_mat)
P_vec = [P_mat(1,:)';P_mat(2,2:3)';P_mat(3,3)];
end

function P_mat = expand(P_vec)
P_vec = squeeze(P_vec);
P_mat = [P_vec(1:3)'; 0 P_vec(4:5)';0 0 P_vec(6)];
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
subplot(2,2,1);
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

    