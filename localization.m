clear
clc
numRobots = 5;
nstates = 2*(numRobots);
state = zeros(numRobots,2,numRobots);
P = zeros(nstates,nstates);

Pinit = [0.5 0; 0 0.5];
Stateinit = [0 0;
        1.5 1.5;
        -1.5 1.5;
        -1.5 -1.5;
        1.5 -1.5];
dists = zeros(numRobots,numRobots);
for i = 1:numRobots
    for j = i:numRobots
        dists(i,j) = sqrt((Stateinit(i,1)-Stateinit(j,1))^2 + (Stateinit(i,2)-Stateinit(j,2))^2);
        dists(j,i) = dists(i,j);
    end
end

Q = [0.1 0;0 0.1];
R = 0.1;
tf = 100;
for i = 1:numRobots
    state(:,:,i) = Stateinit;
    P(:,:,i) = kron(eye(nstates/2),Pinit);
end
trust = ones(numRobots);
for k = 1:tf
    for i = 1:numRobots
        x = state(:,:,i);
        Pr = P(2*i-1:2*i,2*i-1:2*i,i);
        for j = 1:numRobots
            if(i==j)
                continue;
            end
            Pj = P(2*j-1:2*j,2*j-1:2*j,j);
            Pi = P(2*j-1:2*j,2*j-1:2*j,i);
            z = dists(i,j) + normrnd(0,R);            
            zPred = sqrt((state(i,1,i)-state(j,1,j))^2 + (state(i,2,i)-state(j,2,j))^2);
            Res = z-zPred;
            jH = [(state(i,1,i)-state(j,1,j))/zPred (state(i,2,i)-state(j,2,j))/zPred];
            S = jH*Pr*jH' + R;
            K = trust(i,j)*Pr*jH'/S;
            trust(i,j) = trust(i,j) + norm((Pj-Pi),2) ;
            Innov = K*Res;
            state(i,1:2,i) = state(i,1:2,i) + Innov';
            I_KH = eye(2,2) - K*jH;
            Pr = I_KH*Pr*I_KH' + K*R*K';
        end
        P(2*i-1:2*i,2*i-1:2*i,i) = Pr;
    end
end