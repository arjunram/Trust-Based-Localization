truthSaved = 1;
%plotid = 1;
idx = find(update_time(:,1));
update_time1 = update_time(idx,:);
idx = find(update_time1(:,2)==plotid);
update_time1 = update_time1(idx,:);
if(truthSaved)
    %load PropTruthXY.mat
    load UpdateTruthXYAllbots.mat
    %for i = 1:size(StateRec,2)
    %    curTime = StateRec(1,i);
        %err(i,:) = [curTime, StateRec(2*id,i)-trueX(i), StateRec(2*id+1,i)-trueY(i)];
        err = [StateRec(1,:); StateRec(2*plotid,:)-trueX(:,plotid)'; StateRec(2*plotid+1,:)-trueY(:,plotid)'];
    %end
else
    for plotid = 1:num_robots
    %truth = truths(:,:,id);
    for i = 1:size(StateRec,2)
        curTime = StateRec(1,i);
        [d,ix] = min(abs(truths(:,1,plotid)-curTime));
        trueX(i,plotid) = truths(ix,2,plotid);
        trueY(i,plotid) = truths(ix,3,plotid);
        err(i,:) = [curTime, StateRec(2*plotid,i)-trueX(i,plotid), StateRec(2*plotid+1,i)-trueY(i,plotid)];
    end
    end
end
plot(err(1,:)-err(1,1),err(2,:),'r');
hold on
plot(err(1,:)-err(1,1),3*sqrt(CovRec(1,:)),'b');
plot(err(1,:)-err(1,1),-3*sqrt(CovRec(1,:)),'b');
xlabel('Time');
ylabel('X');
%scatter(update_time1(:,1) - update_time1(1,1),zeros(size(update_time1,1),1),5);
figure
plot(err(1,:)-err(1,1),err(3,:),'r');
hold on
%scatter(update_time1(:,1) - update_time1(1,1),zeros(size(update_time1,1),1),5);
plot(err(1,:)-err(1,1),3*sqrt(CovRec(4,:)),'b');
plot(err(1,:)-err(1,1),-3*sqrt(CovRec(4,:)),'b');
xlabel('Time');
ylabel('Y');