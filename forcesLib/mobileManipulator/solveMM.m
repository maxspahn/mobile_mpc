close all;
clear;
clc;

%% figure
% fig1 = figure(1);
% ax1 = axes('Parent', fig1, 'xlim', [-5, 5], 'ylim', [-5, 5], 'zlim', [0, 5]);
% view(ax1, 3);
% grid on;
% hold(ax1, 'on');

fig2 = figure(2);
set(fig2, 'Position', get(0, 'Screensize'))
ax2 = axes('Parent', fig2, 'xlim', [-100, 100]);
axis equal
hold(ax2, 'on');


%%
H = 20;

dt = 0.5;
base_pos = [-3; -2; 0];
arm_pos = [0; 0; 0; -1; 0.5; 1.5; 0];
u_start = zeros(9, 1);
start = [base_pos; arm_pos];
goal_base_pos = [6; 4; pi/2];
goal_arm_pos = [1; 1; -1.5; -1.5; 0.5; 1; 0.2];

goal = [goal_base_pos; goal_arm_pos];

problem.xinit = [start; 0; u_start];
problem.x0 = repmat([start; u_start; 0], H, 1);

obstacles = ones(5 * 4, 1) * -100;
planes = zeros(8 * 9, 1);
%obstacles(1:4) = [6; 2; 0; 1];
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
infPlanes = zeros(3 * 4, 0);
infPlanes(1:4) = [-1, 1, 0, -6]';
infPlanes(5:8) = [1, 0, 0, -4]';
infPlanes(9:12) = [0.5, -1, 0, -6]';

%planes = repmat(planes(1:36), 2, 1);
%planes = repmat(planes(55:63), 8, 1);
% planes(55:63) = planes(64:72);
% planes(37:54) = planes(55:72);



% plane2 = [-1, 0, 0, 11, 2, 0, 1]';
% plane3 = [1, 0, 0, 0, 2, 0, 1]';
% plane4 = [0, 1, 0, 5.5, 3, 0, 5.5]';
%planes = [plane1; plane2; plane3; plane4];
for i=1:0
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
weights = [1, 100, 0, 1000000000, 0, 0];
safetyMargin = 0.0;
for i=1:0
    rectangle('Parent', ax2, 'Position', [obstacles(4 * (i-1) + 1) - obstacles(4 * (i-1) + 4) obstacles(4 * (i-1) + 2) - obstacles(4 * (i-1) + 4) 2 * obstacles(4 * (i-1) + 4) 2 * obstacles(4 * (i-1) + 4)], 'Curvature', 1);
end
plot(goal_base_pos(1), goal_base_pos(2), 'rx');
% ht1 = hgtransform('Parent', ax2);
% posMatrix = makehgtform('translate', [start(1:2)', 0]) * makehgtform('zrotate', start(3));
% set(ht1, 'Matrix', posMatrix);
% rectangle('Parent', ht1, 'Position', [-0.5, -0.5, 1, 1]);

%% Converting obstacles and fill up

r = 0.1;
L = 0.9;
params = repmat([dt, r, L, goal', weights, safetyMargin, planes', obstacles'], 1, H)';
params = repmat([dt, r, L, goal', weights, safetyMargin, infPlanes], 1, H)';
problem.all_parameters = params;

curState = start;
newState = start;
error = 10000;
t = 0;
while 1
    [output, exitflag, info] = mm_MPC(problem);
    %disp(info.res_ineq);
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
    error = norm(curState(1:2) - goal(1:2));
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


    

