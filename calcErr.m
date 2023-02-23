id = 1;
truth = truths(:,:,id);
for i = 1:size(StateRec,2)
    curTime = StateRec(1,i);
    [d,ix] = min(abs(truth(:,1)-curTime));
    trueX = truth(ix,2);
    trueY = truth(ix,3);
    err(i,:) = [curTime, StateRec(2*id,i)-trueX, StateRec(2*id+1,i)-trueY];
end
plot(err(:,1)-err(1,1),err(:,2));
figure
plot(err(:,1)-err(1,1),err(:,3));