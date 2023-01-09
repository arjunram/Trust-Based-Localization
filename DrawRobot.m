function DrawRobot(Xtrue, Xr, Pr,i)
col = ['b';'g';'r'];
p=0.02;                        % percentage of axes size 
a=axis;
l1=(a(2)-a(1))*p;
l2=(a(4)-a(3))*p;
P=[-1 1 0 -1; -1 -1 3 -1];     % basic triangle
theta = Xtrue(3)-pi/2; % rotate to point along x axis (theta = 0)
c=cos(theta);
s=sin(theta);
P=[c -s; s c]*P;               % rotate by theta
P(1,:)=P(1,:)*l1+Xtrue(1);        % scale and shift to x
P(2,:)=P(2,:)*l2+Xtrue(2);
H = plot(P(1,:),P(2,:),col(i),'LineWidth',0.1);% draw
plot(Xtrue(1),Xtrue(2),sprintf('%s+','b'));

plot(Xr(1),Xr(2),'b*');
PlotEllipse(Xr(1:2),Pr,3,i);


end