truthSaved = 1;
id = 4;
idx = find(update_time(:,1));
update_time1 = update_time(idx,:);
idx = find(update_time1(:,2)==id);
update_time1 = update_time1(idx,:);
if(truthSaved)
    %load PropTruthXY.mat
    load UpdateTruthXYAllbots.mat
    %for i = 1:size(StateRec,2)
    %    curTime = StateRec(1,i);
        %err(i,:) = [curTime, StateRec(2*id,i)-trueX(i), StateRec(2*id+1,i)-trueY(i)];
        err = [StateRec(1,:); StateRec(2*id,:)-trueX(:,id)'; StateRec(2*id+1,:)-trueY(:,id)'];
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
plot(err(1,:)-err(1,1),err(2,:));
%hold on
%scatter(update_time1(:,1) - update_time1(1,1),zeros(size(update_time1,1),1),5);
figure
plot(err(1,:)-err(1,1),err(3,:));
%hold on
%scatter(update_time1(:,1) - update_time1(1,1),zeros(size(update_time1,1),1),5);