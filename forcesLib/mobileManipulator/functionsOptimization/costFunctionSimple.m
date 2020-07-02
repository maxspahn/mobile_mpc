function J = costFunctionSimple(z, p, i)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
x = z(1:3);
q = z(4:10);
slack = z(11);
u = z(12:13);
q_dot = z(14:20);
dt = p(1);
r = p(2);
L = p(3);
x_des = p(4:6);
q_des = p(7:13);

wq = p(14);
wx = p(15);
wo = p(16);
wslack = p(17);
wpu = p(18);
wpqdot = p(19);


Wq = diag(ones(size(q, 1), 1) * wq);
Wx = diag(ones(size(x_des, 1), 1) * wx);
% Turning of rotation of base
Wx(3, 3) = wo;

%Wc = diag(ones(size(z(1:10), 1), 1) * wx);

Wslack = wslack;
Wu = diag(ones(size(u, 1), 1) * wpu);
Pqdot = diag(ones(size(q_dot, 1), 1) * wpqdot);
% avoid Turning cost

%contourGoal = computeGoal([x; q], [x_des; q_des], i, dt);

J = (x - x_des)' * Wx * (x - x_des) + ...
    (q - q_des)' * Wq * (q - q_des) + ...
    u' * Wu * u + ...
    q_dot' * Pqdot * q_dot + ...
    slack * Wslack * slack;
end

function contourGoal = computeGoal(curState, goal, i, dt)
    maxSpeedBase = 0.4;
    qdot_max = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100]';
    distanceToGoal = [sqrt((goal(1) - curState(1))^2 + (goal(2) - curState(2))^2); abs(goal(4:10) - curState(4:10))];
    times = distanceToGoal./[maxSpeedBase; qdot_max];
    tmax = times(1);
    pathFraction = dt/tmax;
    contourGoal = pathFraction * i * (goal - curState) * dt;
end

