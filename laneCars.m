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
for i = 1:num_robots
    P(:,:,i) = i*[0.1 0;0 0.2];
    Pt_old(:,:,i) = P(:,:,i);
    trust(i) = 0.5;
end
mu = [0 0 0];
Sigma = [0.01 0 0; 0 0.01 0; 0 0 0.1];

%figure(1);
hold on;
count = 1;
for t = 0:dt:tfinal
    for i = 1:num_robots
        ref = ref_traj(t);
        noise = [0 0 0];%mvnrnd(mu,Sigma,1);[0 0 0];
        x(i) = ref(1) + noise(1);
        y(i) = ref(2) + noise(2);
        theta = ref(3) + noise(3);
        r = rand;
        dist = (1.3^i)*r*sqrt(x(i)^2 + y(i)^2);
        DCM = [cos(theta) sin(theta);-sin(theta) cos(theta)];
        Pt_new = DCM*P(:,:,i)*DCM';
        dist_err(i) = dist - sqrt(ref(1)^2 + ref(2)^2);
        trustdot = -lambda*trust(i) + W1*(1-trust(i))*scale_func(sf*norm((Pt_old(:,:,i)-Pt_new),2) + W2*dist_err(i) + W3*norm(Pt_new,2));
        trust(i) = trust(i) + dt*trustdot;
        trustRec(i,count) = trust(i);
        Pt_old(:,:,i) = Pt_new;
    end
    
    count = count +1;

%     zPred = sqrt((state(2)-y(i)))
%     Res = dist_err;
%     jH = [(state(1)-x(i))/zPred (state(2)-y(i))/zPred];
%             S = jH*Pr*jH' + R;
%             K = trust(i,j)*Pr*jH'/S;
%             trust(i,j) = trust(i,j) + norm((Pj-Pi),2) ;
%             Innov = K*Res;
%             state(i,1:2,i) = state(i,1:2,i) + Innov';
%             I_KH = eye(2,2) - K*jH;
%             Pr = I_KH*Pr*I_KH' + K*R*K';
%    scatter(x,y);
end
%figure;
hold on;
for i = 1:num_robots
plot(trustRec(i,:));
end
xlabel('Time (msec)','FontSize',16);
ylabel('Trust','FontSize',16);
legend({'Agent 1','Agent 2','Agent 3'},'FontSize',16,'Location','best');

function scale = scale_func(val)
scale = 1/(exp(val));
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