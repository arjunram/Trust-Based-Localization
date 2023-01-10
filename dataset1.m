clear
load Set1.mat;

odomPointer = 1;
measPointer = 1;
posState = pos_est(2:4,:);
posCov = zeros(3,3,num_robots);
StateRec = zeros(3*num_robots+1,1);
StateRec(1,1) = max(pos_est(1,:));
StateRec(2:end,1) = reshape(posState,[],1);
CovRec(:,1) = posCov(:);
%odomTime = zeros(num_robots,1);
odomTime = pos_est(1,:)';

while(odomPointer<size(robot_odom,1) || measPointer<size(robot_meas,1))
%while(odomPointer<10 || measPointer<10)    
    if(robot_odom(odomPointer,1)<=robot_meas(measPointer,1) || (measPointer==size(robot_meas,1) && odomPointer<size(robot_odom,1)))
        robotId = robot_odom(odomPointer,2);
        timeTag = robot_odom(odomPointer,1); 
        dt = timeTag - odomTime(robotId);
        odomTime(robotId) = timeTag;
        [posState(:,robotId),posCov(:,:,robotId)] = propagate(robot_odom(odomPointer,3:4),dt,posState(:,robotId),posCov(:,:,robotId));
        StateRec(1,end+1) = timeTag;
        StateRec(2:end,end) = reshape(posState,[],1);
        CovRec(:,end+1) = posCov(:);
        odomPointer = odomPointer + 1;
    elseif(robot_odom(odomPointer,1)>robot_meas(measPointer,1) || (measPointer<size(robot_meas,1) && odomPointer==size(robot_odom,1)))
        robotId = robot_meas(measPointer,2);
        measId = robot_meas(measPointer,3);
        if(measId<6)
            [posState(:,robotId),posCov(:,:,robotId)] = update(robot_meas(measPointer,4:5),posState(:,robotId),posState(:,measId),posCov(:,:,robotId),posCov(:,:,measId));
            StateRec(1,end+1) = robot_meas(measPointer,1);
            StateRec(2:end,end) = reshape(posState,[],1);
            CovRec(:,end+1) = posCov(:);
        end
        measPointer = measPointer + 1;
    end
end

function [pose,cov] = propagate(odom,dt,pose,cov)
    if(dt<0) 
        return
    end
    vx = odom(1)*cos(pose(3));
    vy = odom(1)*sin(pose(3));
    pose(1:2) = pose(1:2) + [vx;vy]*dt;
    pose(3) = pose(3) + odom(2)*dt;
end

function [pose,cov] = update(meas,pose,targetPose,cov,targetCov)
end