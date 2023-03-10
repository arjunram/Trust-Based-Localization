%%%%%%%% Cavariance elipse ( Eig values) %%%%%%%%
function eH = PlotEllipse(x,P,nSigma,i)
col = ['b';'g';'r'];
eH = [];
P = P(1:2,1:2); % we are only interested in x and y part
x = x(1:2);
if(~all(diag(P)==0))
    [V,D] = eig(P);
    y = nSigma*[cos(0:0.1:2*pi);sin(0:0.1:2*pi)];
    el = V*sqrt(D)*y;
    el = [el el(:,1)]+repmat(x,1,size(el,2)+1);
    eH = line(el(1,:),el(2,:),'Color',col(i));
end
end