close all;
clear;
clc;

%% figure
% fig1 = figure(1);
% ax1 = axes('Parent', fig1, 'xlim', [0, 100]);
% hold(ax1, 'on');

fig2 = figure(2);
set(fig2, 'Position', get(0, 'Screensize'));
set(fig2, 'WindowStyle', 'docked');
ax2 = axes('Parent', fig2, 'xlim', [-20, 20]);
axis equal
hold(ax2, 'on');


%%
H = 20;

dt = 0.5;
base_pos = [0; 0; 0];
arm_pos = [0; 0; 0; -1; 0; 1; 0];
u_start = zeros(9, 1);
start = [base_pos; arm_pos];
goal_base_pos = [5; 0; 0];
goal_arm_pos = [1; 0; -1.5; -1.5; 0.5; 1; 0.2];

goal = [goal_base_pos; goal_arm_pos];

problem.xinit = [start; 0; u_start];
problem.x0 = repmat([start; u_start; 0], H, 1);

obstacles = ones(20 * 4, 1) * -100;
obstacles(1:4) = [1; -3; 2.1; 1];
obstacles(5:8) = [2; 0.1; 0; 1];
% obstacles(9:12) = [8; 7; 0; 0.1];

% wq, wx, wo, wslack, wpu, wpqdot, 
weights = [1, 10, 1, 10000, 0, 10];
safetyMargin = 0.5;
for i=1:2
    rectangle('Parent', ax2, 'Position', [obstacles(4 * (i-1) + 1) - obstacles(4 * (i-1) + 4) obstacles(4 * (i-1) + 2) - obstacles(4 * (i-1) + 4) 2 * obstacles(4 * (i-1) + 4) 2 * obstacles(4 * (i-1) + 4)], 'Curvature', 1);
end
plot(goal_base_pos(1), goal_base_pos(2), 'rx');
% ht1 = hgtransform('Parent', ax2);
% posMatrix = makehgtform('translate', [start(1:2)', 0]) * makehgtform('zrotate', start(3));
% set(ht1, 'Matrix', posMatrix);
% rectangle('Parent', ht1, 'Position', [-0.5, -0.5, 1, 1]);



%% Converting obstacles and fill up

r = 0.08;
L = 0.544;

%% Debugging
params = repmat([dt, r, L, goal', weights, safetyMargin, obstacles'], 1, H)';
problem.all_parameters = params;
collStruct.nbObstacles = 20;
collStruct.nbSpheres = 3;
collStruct.nbSelfCollision = 0;
collStruct.nbPlanes = 0;
collStruct.nbInfPlanesEachSphere = 0;
collStruct.dimPlane = 9;
collStruct.dimInfPlane = 4;
collStruct.dimObstacle = 4;
ineq = obstacleAvoidanceSimple(problem.xinit, params(1:200), collStruct);
cost = costFunctionSimple(problem.xinit, params(1:200));

curState = start;
newState = start;
error = 10000;
t = 0;
times = [];
while 1
    [output, exitflag, info] = mm_MPC(problem);
    %ForcesDumpProblem(problem, tag);
    %disp(info.res_ineq);
    times = [times, info.solvetime];
    disp(exitflag);
    curState = output.x02(1:10);
    curU = output.x01(12:13);
    slack = output.x02(11);
    disp(slack);
    problem.xinit = output.x02;
    problem.x0 = [output.x01;
        output.x02;...
        output.x03;...
        output.x04;...
        output.x05;...
        output.x06;...
        output.x07;...
        output.x08;...
        output.x09;...
        output.x10;...
        output.x11;...
        output.x12;...
        output.x12];
    problem.x0 = repmat(output.x01, H, 1);
    
    
    plot(ax2, curState(1), curState(2), 'ro');
    pause(0.1);
    
    oldError = error;
    error = norm(curState - goal);
    %disp(error);
    if error < 0.1 || oldError <= 0.1 * error
        break;
    end
    t = t + dt;
  
end

function x_base_next = dynamics_mm(u, x_base, dt, r, L)
x_base_next = [x_base(1) + r/2 * (u(1) + u(2)) * cos(x_base(3)) * dt;
    x_base(2) + r/2 * (u(1) + u(2)) * sin(x_base(3)) * dt;
    x_base(3) + r/L * (u(1) - u(2)) * dt];
end

function [x, y, z] = planeEquation(plane)
    v1 = plane(4:6) - plane(1:3);
    v2 = plane(7:9) - plane(1:3);
    s = 0:0.01:1;
    t = 0:0.01:1;
    values = zeros(size(s, 2), size(t, 2), 3);
    for i=1:size(s, 2)
        for j=1:size(t, 2)
            values(i, j, :) = plane(1:3) + v1 * s(i) + v2 * t(j);
        end
    end
    x = values(:, :, 1);
    y = values(:, :, 2);
    z = values(:, :, 3);
end

function [x, y, z, n] = infPlaneEquation(infPlane)
    a = infPlane(1:3);
    d = infPlane(4);
    if (a(2) == 0)
        y = -10:0.1:10;
        x = (d - a(2) * y) / a(1);
    else
        x = -10:0.1:10;
        y = (d - a(1) * x )/ a(2);
    end
    z = zeros(size(x));
    n0 = [x(50), y(50), z(50)]';
    n1 = n0 + a';
    n = [n0, n1];
end


    

