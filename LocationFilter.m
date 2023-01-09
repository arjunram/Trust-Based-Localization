clc;
clear;
x_pos = [0; 0];
rob_pos = [10 -20 -30;
           10 0 30]';
x_cov = [5 0;
        0 5];
rob_cov = zeros(2,2,3);
rob_cov(:,:,1) = [1 0;
                  0 1];
rob_cov(:,:,2) = [10 0;
                  0 10];
rob_cov(:,:,3) = [5 0;
                  0 5];
trust = [1;0.1;0.5];
%trust = [1;1;1];
Q = [0.01 0;0 0.01];
R = 0.001;

nsteps = 1000;
x_true = [0;0];
nMonte = 100;

for k = 1:nMonte
    x_pos = [0; 0];

    x_cov = [5 0;
        0 5];
for t = 1:nsteps

    CtrlNoise = randn(2,1);
    u = sqrt(Q)*CtrlNoise;
    x_bar = x_pos + u;
    P_bar = x_cov + Q;
    for i = 1:3
        meas_noise = normrnd(0,R);
        range_true = sqrt((rob_pos(i,1)-x_true(1))^2 + (rob_pos(i,2)-x_true(2))^2);
        range_meas = range_true + sqrt(R)*meas_noise;
        range_pred = sqrt((rob_pos(i,1)-x_bar(1))^2 + (rob_pos(i,2)-x_bar(2))^2);
        H = [(x_bar(1)-rob_pos(i,1)) (x_bar(2)-rob_pos(i,2))]/range_pred;

        innov = range_meas - range_pred;
        %S = H*P_bar*H' + R/trust(i);
        S = H*P_bar*H' + R;
        K = trust(i)*P_bar*H'/S;
        %K = P_bar*H'/S;
        x_bar = x_bar + K*innov;
        P_bar = (eye(2) - K*H)*P_bar*(eye(2) - K*H)' + K*R*K';
    end
    x_pos = x_bar;
    x_cov = P_bar;
xposRec(:,t,k) = x_pos;
PRec(:,t,k) = diag(x_cov);
NEES(t,k) = x_pos'*inv(3*x_cov)*x_pos/2;
end
end

figure(1);
subplot(2,1,1);
plot(1:nsteps,squeeze(xposRec(1,:,:)),'b');
hold on;
plot(1:nsteps,3*sqrt(squeeze(PRec(1,:,:))),'r');
plot(1:nsteps,-3*sqrt(squeeze(PRec(1,:,:))),'r');
title('x');
subplot(2,1,2);
plot(1:nsteps,squeeze(xposRec(2,:,:)),'b');
hold on;
plot(1:nsteps,3*sqrt(squeeze(PRec(2,:,:))),'r');
plot(1:nsteps,-3*sqrt(squeeze(PRec(2,:,:))),'r');
title('y');

figure;
plot(1:nsteps,mean(NEES,2),'b')