clc
clear

tfinal = 15;
dt = 0.1;
lambda = 1;
W1 = 1;
W2 = 1.2;
W3 = 0.1;
num_robots = 2;
P = [0.1 0;0 0.2];
Pt_old = P;
mu = [0 0 0];
Sigma = [0.01 0 0; 0 0.01 0; 0 0 0.1];
trust = 0.5;
figure(1);
hold on;
count = 1;
for t = 0:dt:tfinal
    ref = ref_traj(t);
    noise = [0 0 0];%mvnrnd(mu,Sigma,1);[0 0 0];
    x = ref(1) + noise(1);
    y = ref(2) + noise(2);
    theta = ref(3) + noise(3);
    DCM = [cos(theta) sin(theta);-sin(theta) cos(theta)];
    Pt_new = DCM*P*DCM';
    Pos_err = sqrt((ref(1)-x)^2 + (ref(2)-y)^2);
    trustdot = -lambda*trust + scale_func(W1*norm((Pt_old-Pt_new),2) + W2*Pos_err + W3*norm(Pt_new,2));
    trust = trust + dt*trustdot;
    trustRec(count) = trust;
    count = count +1;
    Pt_old = Pt_new;
    scatter(x,y);
end
figure;
plot(trustRec);

function scale = scale_func(val)
scale = 2/(exp(val)) - 1
end

function ref = ref_traj(t)
rad = 1;
omega = 2*pi*0.1;
if t>10
    rad = 2;
end
ref(1) = rad*cos(omega*t);
ref(2) = rad*sin(omega*t);
ref(3) = omega*t + pi/2;
if(ref(3)>2*pi) ref(3) = ref(3)-2*pi; end
if(ref(3)<0) ref(3) = ref(3)+2*pi; end
end