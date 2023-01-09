clear
load Dataset1.mat;

num_robots = 5;

pos_est = zeros(num_robots,3);
truthSize = zeros(num_robots,1);
odomSize = zeros(num_robots,1);
measSize = zeros(num_robots,1);
% Set starting positions to be completely known
for i = 1:num_robots
    temp_var = eval(strcat('robot',num2str(i),'_truth'));
    pos_est(i,:) = temp_var(1,2:4);
    truthSize(i) = size(temp_var,1);
    temp_var = eval(strcat('robot',num2str(i),'_odom'));
    odomSize(i) = size(temp_var,1);
    temp_var = eval(strcat('robot',num2str(i),'_meas'));
    measSize(i) = size(temp_var,1);
end

truths = zeros(max(truthSize),4,num_robots);
odoms = zeros(max(odomSize),3,num_robots);
meas = zeros(max(measSize),4,num_robots);

for i = 1:num_robots
    temp_var = eval(strcat('robot',num2str(i),'_truth'));
    truths(1:truthSize(i),:,i) = temp_var;
    temp_var = eval(strcat('robot',num2str(i),'_odom'));
    odoms(1:odomSize(i),:,i) = temp_var;
    temp_var = eval(strcat('robot',num2str(i),'_meas'));
    meas(1:measSize(i),:,i) = temp_var;
end

robot_meas = zeros(1,5);
count = 0;
for i = 1:num_robots
    for j = 1:measSize(i)
        count = count + 1;
        robot_meas(count,1) = meas(j,1,i);
        robot_meas(count,2) = i;
        robot_meas(count,3:5) = meas(j,2:4,i);
    end
end
robot_meas = sortrows(robot_meas);

robot_odom = zeros(1,4);
count = 0;
for i = 1:num_robots
    for j = 1:odomSize(i)
        count = count + 1;
        robot_odom(count,1) = odoms(j,1,i);
        robot_odom(count,2) = i;
        robot_odom(count,3:4) = odoms(j,2:3,i);
    end
end
robot_odom = sortrows(robot_odom);

save('Set1.mat','num_robots','truthSize','truths','meas','measSize','odoms','odomSize','landmarks','robot_meas','robot_odom');