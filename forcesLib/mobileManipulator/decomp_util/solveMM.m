close all;
clear;
clc;

addpath('../functionsOptimization');
addpath('../dyn_model_panda');
addpath('../distanceFunctions');

%% figure
% fig1 = figure(1);
% ax1 = axes('Parent', fig1, 'xlim', [-5, 5], 'ylim', [-5, 5], 'zlim', [0, 5]);
% view(ax1, 3);
% grid on;
% hold(ax1, 'on');

fig2 = figure(2);
set(fig2, 'Position', get(0, 'Screensize'));
set(fig2, 'WindowStyle', 'docked');
ax2 = axes('Parent', fig2, 'xlim', [-5, 5], 'ylim', [-5, 5]);
hold(ax2, 'on');


%%
H = 20;

dt = 2.5;
base_pos = [0; 0; 2.5];
arm_pos = [0; 0; 0; -1; 0; 1; 0];
u_start = zeros(9, 1);
start = [base_pos; arm_pos];
goal_base_pos = [0; -4; 0];
goal_arm_pos = [1; 0; -1.5; -1.5; 0.5; 1; 0.2];

goal = [goal_base_pos; goal_arm_pos];

problem.xinit = [start; 0; u_start];
problem.x0 = repmat([start; u_start; 0], H, 1);

obstacles = ones(20 * 4, 1) * -100;
movingObstacles = ones(1 * 7, 1) * -100;
planes = zeros(1 * 9, 1);
%obstacles(1:4) = [1; -3; 2.1; 1];
%obstacles(5:8) = [11; 10; 0; 2];
% obstacles(9:12) = [8; 7; 0; 0.1];
line1 = [3, 15, 0, 12, 8, 0]';
planes(1:9) = [-2.5, -2, 0, 2.5, -2, 0, -2.5, -2, 2]';
planes(10:18) = [-2.5, -3, 0, 2.5, -3, 0, -2.5, -3, 2]';
planes(19:27) = [6, -2, 0, 6, -7, 0, 6, -2, 2]';
planes(28:36) = [5, -2, 0, 5, -7, 0, 5, -2, 2]';
planes(37:45) = [-2, -5.5, 0, 0, -5.5, 0, -2, -5.5, 2]';
planes(46:54) = [-2, -7.5, 0, 0, -7.5, 0, -2, -7.5, 2]';
planes(55:63) = [-2, -5.5, 0, -2, -7.5, 0, -2, -5.5, 2]';
planes(64:72) = [0, -5.5, 0, 0, -7.5, 0, 0, -5.5, 2]';
planes = [-1.5, 5, 0, 1.5, 5, 0, -1.5, 5, 2]';
planes = [0, 0, 5, 5, 0, 5, 0, 5, 5]';
defaultInfPlane = [0, 0, 1, -3];
infPlanes = repmat(defaultInfPlane', 60, 1);
infPlanes(1:4) = [-1, -1, 0, -4]';
infPlanes(5:8) = [-1, 0, 0, -4]';
% infPlanes(9:12) = [0, 1, 0, -2]';
% infPlanes(9:12) = [0.5, -1, 0, -6]';

%planes = repmat(planes(1:36), 2, 1);
%planes = repmat(planes(55:63), 8, 1);
% planes(55:63) = planes(64:72);
% planes(37:54) = planes(55:72);



% plane2 = [-1, 0, 0, 11, 2, 0, 1]';
% plane3 = [1, 0, 0, 0, 2, 0, 1]';
% plane4 = [0, 1, 0, 5.5, 3, 0, 5.5]';
%planes = [plane1; plane2; plane3; plane4];
for i=1:1
    [xp, yp, zp] = planeEquation(planes(9 * (i -1) + 1: 9 * (i -1) + 9));
    plot(ax2, xp, yp);
end
for i = 1:3
    [xp, yp, zp, np] = infPlaneEquation(infPlanes(4 * (i - 1) + 1: 4 * (i - 1) + 4));
    plot(ax2, xp, yp);
    plot(ax2, np(1,:), np(2,:), 'bx');
end
%plot(ax2, [line1(1); line1(4)], [line1(2); line1(5)]);

% wq, wx, wo, wslack, wpu, wpqdot, 
weights = [1, 1, 0.0, 10000, 0.5, 10];
safetyMargin = 0.5;
for i=1:0
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
param = [dt, r, L, goal', weights, safetyMargin, infPlanes', movingObstacles'];
params = repmat(param, 1, H)';
%params = repmat([dt, r, L, goal', weights, safetyMargin, obstacles'], 1, H)';
problem.all_parameters = params;
collStruct.nbObstacles = 0;
collStruct.nbSpheres = 3;
collStruct.nbSelfCollision = 0;
collStruct.nbPlanes = 0;
collStruct.nbInfPlanesEachSphere = 20;
collStruct.dimPlane = 9;
collStruct.dimInfPlane = 4;
collStruct.dimObstacle = 4;
collStruct.dimMovingObstacle = 7;
collStruct.nbMovingObstacles = 1;
% ineq = obstacleAvoidanceSimple(problem.xinit, param', collStruct, 1);
% cost = costFunctionSimple(problem.xinit, param');

curState = start;
newState = start;
error = 10000;
t = 0;
while t < 200
    movingObstacles = [sin(t/20) * 1.5 - 1.50, -2, 0, cos(t/20) * 1.5, 0, 0, 0.5]';
    param = [dt, r, L, goal', weights, safetyMargin, infPlanes', movingObstacles'];
    ineq = obstacleAvoidanceSimple(problem.xinit, param', collStruct, 1);
    plot(ax2, movingObstacles(1), movingObstacles(2), 'gx');
    problem.all_parameters = params;
    params = repmat(param, 1, H)';
    [output, exitflag, info] = mm_MPC(problem);
    %ForcesDumpProblem(problem, tag);
    %disp(info.res_ineq);
    disp(exitflag);
    disp(info.solvetime);
    curState = output.x01(1:10);
    fn = fieldnames(output);
    fss = zeros(17, 2);
    for i=3:numel(fn)
        if (isnumeric(output.(fn{i})))
            futureState = output.(fn{i});
            fss(i-2, :) = futureState(1:2);
        end
    end
    
    curU = output.x01(12:13);
    slack = output.x01(11);
    problem.xinit = output.x02;
    problem.x0 = [output.x01;...
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
        output.x13;...
        output.x14;...
        output.x15;...
        output.x16;...
        output.x17;...
        output.x18;...
        output.x19;...
        output.x20];
    %problem.x0 = repmat(output.x01, H, 1);
    
    
    plot(ax2, curState(1), curState(2), 'ro');
    for i = 1:17
        plot(ax2, fss(i,1), fss(i,2), 'bx');
    end
    pause(0.1);
    
    oldError = error;
    error = norm([curState(1:2); curState(4:end)] - [goal(1:2); goal(4:end)]);
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


    

