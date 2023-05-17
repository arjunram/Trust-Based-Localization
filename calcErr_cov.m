truthSaved = 1;
robot = 2;
num_robots = 5;
idx = find(update_time(:,1));
update_time1 = update_time(idx,:);
idx = find(update_time1(:,2)==robot);
update_time1 = update_time1(idx,:);
if(truthSaved)
    %load PropTruthXY.mat
    load UpdateTruthXYAllbots.mat
    %for i = 1:size(StateRec,2)
    %    curTime = StateRec(1,i);
        %err(i,:) = [curTime, StateRec(2*id,i)-trueX(i), StateRec(2*id+1,i)-trueY(i)];
        err = [StateRec(1,:); StateRec(2*robot,:)-trueX(:,robot)'; StateRec(2*robot+1,:)-trueY(:,robot)'];
    %end
else
    for id = 1:num_robots
    %truth = truths(:,:,id);
    for i = 1:size(StateRec,2)
        curTime = StateRec(1,i);
        [d,ix] = min(abs(truths(:,1,id)-curTime));
        trueX(i,id) = truths(ix,2,id);
        trueY(i,id) = truths(ix,3,id);
        err(i,:) = [curTime, StateRec(2*id,i)-trueX(i,id), StateRec(2*id+1,i)-trueY(i,id)];
    end
    end
end
plot(err(1,:)-err(1,1),err(2,:),'r');
hold on
plot(err(1,:)-err(1,1),3*sqrt(CovRec((robot-1)*(2*num_robots)^2+4*num_robots*(robot-1)+2*robot-1,:)),'b');
plot(err(1,:)-err(1,1),-3*sqrt(CovRec((robot-1)*(2*num_robots)^2+4*num_robots*(robot-1)+2*robot-1,:)),'b');

%scatter(update_time1(:,1) - update_time1(1,1),zeros(size(update_time1,1),1),5);
figure
plot(err(1,:)-err(1,1),err(3,:),'r');
hold on
%scatter(update_time1(:,1) - update_time1(1,1),zeros(size(update_time1,1),1),5);
plot(err(1,:)-err(1,1),3*sqrt(CovRec((robot-1)*(2*num_robots)^2+4*num_robots*(robot-0.5)+2*robot,:)),'b');
plot(err(1,:)-err(1,1),-3*sqrt(CovRec((robot-1)*(2*num_robots)^2+4*num_robots*(robot-0.5)+2*robot,:)),'b');