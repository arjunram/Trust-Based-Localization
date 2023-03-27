clear
load Set1.mat;
TrustInit = 0.8;
odomPointer = 1;
measPointer = 1;
posState = pos_est(2:3,:); % x and y for each robot
posCov = zeros(2,2,num_robots); %x and y covariance of each robot
StateRec = zeros(2*num_robots+1,1); % time in first index, followed by x and y of each robot
StateRec(1,1) = max(pos_est(1,:)); %last starting time of the initial robot positions
StateRec(2:end,1) = reshape(posState,[],1);
CovRec(:,1) = posCov(:);
%odomTime = zeros(num_robots,1);
odomTime = pos_est(1,:)';
measTime = zeros(5,5);
oldCov = zeros(5,5,3);
Q = 0.001*eye(2);
R = 0.1;%10*eye(2);
update_time(1,:) = [0 0];
while(odomPointer<size(odom_xy,1) || measPointer<size(robot_meas,1))
%while(odomPointer<10 || measPointer<10)    
    if(odom_xy(odomPointer,1)<=robot_meas(measPointer,1) || (measPointer==size(robot_meas,1) && odomPointer<size(odom_xy,1)))
        robotId = odom_xy(odomPointer,2);
        timeTag = odom_xy(odomPointer,1);
        dt = timeTag - odomTime(robotId);
        odomTime(robotId) = timeTag;
        [posState(:,robotId),posCov(:,:,robotId)] = propagate(odom_xy(odomPointer,3:4),dt,posState(:,robotId),posCov(:,:,robotId),Q);
        StateRec(1,end+1) = timeTag;
        StateRec(2:end,end) = reshape(posState,[],1);
        CovRec(:,end+1) = posCov(:);
        odomPointer = odomPointer + 1;
    elseif(odom_xy(odomPointer,1)>robot_meas(measPointer,1) || (measPointer<size(robot_meas,1) && odomPointer==size(odom_xy,1)))
        robotId = robot_meas(measPointer,2);
        measId = robot_meas(measPointer,3);
        if(measId<6)
            if(measTime(robotId,measId)==0)
                trust(robotId,measId) = TrustInit;
                measTime(robotId,measId) = robot_meas(measPointer,1);
                oldCov(robotId,measId,1) = posCov(1,1,measId);
                oldCov(robotId,measId,2) = posCov(1,2,measId);
                oldCov(robotId,measId,3) = posCov(2,2,measId);

            else
                cov_old = zeros(2);
                cov_old(1,1) = oldCov(robotId,measId,1);
                cov_old(1,2) = oldCov(robotId,measId,2);
                cov_old(2,1) = oldCov(robotId,measId,2);
                cov_old(2,2) = oldCov(robotId,measId,3);
                dt = robot_meas(measPointer,1) - measTime(robotId,measId);
                trust(robotId,measId) = trustCalc(trust(robotId,measId),robot_meas(measPointer,4:5),posState(:,measId),posCov(:,:,measId));
                oldCov(robotId,measId,1) = posCov(1,1,measId);
                oldCov(robotId,measId,2) = posCov(1,2,measId);
                oldCov(robotId,measId,3) = posCov(2,2,measId);
                measTime(robotId,measId) = robot_meas(measPointer,1);

            end
            [posState(:,robotId),posCov(:,:,robotId)] = update(robot_meas(measPointer,4:5),posState(:,robotId),posState(:,measId),posCov(:,:,robotId),posCov(:,:,measId),zeros(2),zeros(2),R);
            StateRec(1,end+1) = robot_meas(measPointer,1);
            StateRec(2:end,end) = reshape(posState,[],1);
            update_time(measPointer,:) = [robot_meas(measPointer,1) robotId];
            
            CovRec(:,end+1) = posCov(:);
        end
        measPointer = measPointer + 1;
    end
end

function [pose,cov] = propagate(odom,dt,pose,cov,Q)
    if(dt<0) 
        return
    end
    %vx = odom(1)*cos(pose(3));
    %vy = odom(1)*sin(pose(3));
    vx = odom(1);
    vy = odom(2);
    pose(1:2) = pose(1:2) + [vx;vy]*dt;
    %pose(3) = pose(3) + odom(2)*dt;
    cov = cov + Q*dt;
end

function trust = trustCalc(trust,robot_meas,cov_old,cov,dt)
%trustdot = -lambda*trust(i) + W1*(1-trust(i))*scale_func(sf*norm((Pt_old(:,:,i)-Pt_new),2) + W2*innov + W3*norm(Pt_new,2));
lambda = 0.3;
W1 = 1;
W2 = 1.2;
W3 = 0.1;
sf = 1;

trust = trust*exp(-lambda*dt);
innov = 0;
%W1*(1-trust)*scale_func(sf*norm(cov_old - cov,2) + W2*innov + W3*norm(cov,2));
trust = 1;
end

function scale = scale_func(val)
scale = 1/(exp(val));
end

function [state,cov] = update(meas,pose,targetPose,cov,targetCov, R, trust)
range = meas(1);
%bearing = meas(2);
range_pred = sqrt((targetPose(1)-pose(1))^2 + (targetPose(2)-pose(2))^2);
%bearing_pred = atan2(targetPose(2)-pose(2),targetPose(1)-pose(1));
innov = range - range_pred;%[range - range_pred; bearing - bearing_pred];
state = pose;%[pose;targetPose];
stateCov = cov;%[cov sigma_ij; sigma_ji targetCov];
[jr, jtheta] = GetObsJacs(pose,targetPose);
H = jr(1:2);%[jr(1:2);jtheta(1:2)];
S = H*stateCov*H' + R + norm(targetCov);
K = trust*stateCov*H'/S;
state = state + K*innov;
cov = (eye(2) - K*H)*stateCov*(eye(2)- K*H)' + K*R*K';
end

function [jr,jtheta] = GetObsJacs(pose,targetpose)
%jr = zeros(1,4);jtheta = zeros(1,4);
dif = pose - targetpose;
r = norm(dif);%range
jr= [(dif)'/r (-dif)'/r];
jtheta = [-dif(2) dif(1) dif(2) -dif(1)]/r^2;
end
