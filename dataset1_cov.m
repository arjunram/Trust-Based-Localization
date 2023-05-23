clear
%load Set1.mat;
load Set1_10p_robot3.mat;
plotid = 1;
TrustInit = 0.8;
odomPointer = 1;
measPointer = 1;
num_robots = 5;
posState = pos_est(2:3,:); % x and y for each robot
posCov = zeros(2*num_robots,2*num_robots,num_robots); %x and y covariance of each robot
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
        [posState(:,robotId),posCov(2*robotId-1:2*robotId,2*robotId-1:2*robotId,robotId)] = propagate(odom_xy(odomPointer,3:4),dt,posState(:,robotId),posCov(2*robotId-1:2*robotId,2*robotId-1:2*robotId,robotId),Q);
        StateRec(1,end+1) = timeTag;
        StateRec(2:end,end) = reshape(posState,[],1);
        CovRec(:,end+1) = posCov(:);
        odomPointer = odomPointer + 1;
    elseif(odom_xy(odomPointer,1)>robot_meas(measPointer,1) || (measPointer<size(robot_meas,1) && odomPointer==size(odom_xy,1)))
        robotId = robot_meas(measPointer,2);
        measId = robot_meas(measPointer,3);
        if(measId<6)
            measTime(robotId,measId) = robot_meas(measPointer,1);
            dt = robot_meas(measPointer,1) - measTime(robotId,measId);
            [posState(:,robotId),posCov(:,:,robotId)] = update(robotId,measId,num_robots,robot_meas(measPointer,4:5),posState(:,robotId),posState(:,measId),posCov(:,:,robotId),posCov(:,:,measId),R);
            StateRec(1,end+1) = robot_meas(measPointer,1);
            StateRec(2:end,end) = reshape(posState,[],1);
            update_time(measPointer,:) = [robot_meas(measPointer,1) robotId];
            
            CovRec(:,end+1) = posCov(:);
        end
        measPointer = measPointer + 1;
    end
end

calcErr_cov

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


function scale = scale_func(val)
scale = 1/(exp(val));
end

function [state,cov] = update(robotId,measId,num_robots,meas,pose,targetPose,cov,targetCov, R)
range = meas(1);
%bearing = meas(2);
range_pred = sqrt((targetPose(1)-pose(1))^2 + (targetPose(2)-pose(2))^2);
%bearing_pred = atan2(targetPose(2)-pose(2),targetPose(1)-pose(1));
innov = range - range_pred;%[range - range_pred; bearing - bearing_pred];
state = pose;%[pose;targetPose];
stateCov = cov;%[cov sigma_ij; sigma_ji targetCov];
[jr, jtheta] = GetObsJacs(pose,targetPose);
H = zeros(1,2*num_robots);
H(2*robotId-1:2*robotId) = jr(1:2);%[jr(1:2);jtheta(1:2)];
S = H*stateCov*H' + R + norm(targetCov(2*measId-1:2:measId,2*measId-1:2:measId));
K = stateCov*H'/S;
state = state + K(2*robotId-1:2*robotId)*innov;
cov = (eye(2*num_robots) - K*H)*stateCov*(eye(2*num_robots)- K*H)' + K*R*K';
end

function [jr,jtheta] = GetObsJacs(pose,targetpose)
%jr = zeros(1,4);jtheta = zeros(1,4);
dif = pose - targetpose;
r = norm(dif);%range
jr= [(dif)'/r (-dif)'/r];
jtheta = [-dif(2) dif(1) dif(2) -dif(1)]/r^2;
end
