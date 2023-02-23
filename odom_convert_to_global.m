clear
clc
load Set1.mat

odom_xy = zeros(size(robot_odom));
for i = 1:size(robot_odom,1)
    id = robot_odom(i,2);
    odom_time = robot_odom(i,1);
    [d,ix] = min(abs(truths(:,1,id)-odom_time));
    true_angle = truths(ix,4,id);
    odom_xy(i,1) = robot_odom(i,1);
    odom_xy(i,2) = robot_odom(i,2);
    odom_xy(i,3:4) = robot_odom(i,3)*[cos(true_angle) sin(true_angle)];
end
