close all;
clear;
clc;

addpath('../distanceFunctions');
tag = 'dumpedFiles/simplempc_O3BMONJHPKF';
pathForces = '/home/mspahn/develop/forces';
pathCasadi = '/home/mspahn/develop/casadi';

forcesPath = genpath(pathForces);
casadiPath = genpath(pathCasadi);
addpath(forcesPath);
addpath(casadiPath);

addpath('costFunctions');
addpath('ineqFunctions');


%% figure
% fig1 = figure(1);
% ax1 = axes('Parent', fig1, 'xlim', [-5, 5], 'ylim', [-5, 5], 'zlim', [0, 5]);
% view(ax1, 3);
% grid on;
% hold(ax1, 'on');

fig2 = figure(2);
set(fig2, 'Position', get(0, 'Screensize'));
set(fig2, 'WindowStyle', 'docked');
ax2 = axes('Parent', fig2, 'xlim', [-50, 50], 'ylim', [-50, 50]);
hold(ax2, 'on');


%%
H = 15;

dt2 = 3.0;
dt1 = 0.5;
base_pos = [0; 0; 0.0];
arm_pos = [0; 0; 0; -1; 0; 1; 0];
u_start = zeros(9, 1);
start = [base_pos; arm_pos];
x_spline = -1 * (0.0:0.5:10.0);
y_spline = 1 * (0.0:-1.5:-30.0);
o_spline = ones(size(x_spline)) * atan(y_spline(3)/x_spline(3));
base_spline = [x_spline; y_spline; o_spline];
o_des = 0.0;
goal = base_spline(:,end);
goal_arm_pos = [1; 0; -1.5; -1.5; 0.5; 1; 0.2];
goal = [goal; goal_arm_pos];



problem.xinit = [start; 0; u_start];
problem.x0 = repmat([start; u_start; 0], H, 1);

obstacles = ones(20 * 4, 1) * -100;
movingObstacles = ones(5 * 7, 1) * -100;
planes = zeros(1 * 9, 1);
%obstacles(1:4) = [1; -3; 2.1; 1];
%obstacles(5:8) = [11; 10; 0; 2];
% obstacles(9:12) = [8; 7; 0; 0.1];
line1 = [3, 15, 0, 12, 8, 0]';
defaultInfPlane = [0, 0, 1, -3];
infPlanes = repmat(defaultInfPlane', 60, 1);
infPlanes(1:4) = [-1, -1, 0, -8]';
infPlanes(5:8) = [-1, 0, 0, -10]';
infPlanes(9:12) = [0, 1, 0, -40.5]';
infPlanes(13:16) = [1, 1, 0, -16]';

%planes = repmat(planes(1:36), 2, 1);
%planes = repmat(planes(55:63), 8, 1);
% planes(55:63) = planes(64:72);
% planes(37:54) = planes(55:72);



% plane2 = [-1, 0, 0, 11, 2, 0, 1]';
% plane3 = [1, 0, 0, 0, 2, 0, 1]';
% plane4 = [0, 1, 0, 5.5, 3, 0, 5.5]';
%planes = [plane1; plane2; plane3; plane4];
htpl = hgtransform('Parent', ax2, 'Matrix', eye(4));

for i=1:1
    [xp, yp, zp] = planeEquation(planes(9 * (i -1) + 1: 9 * (i -1) + 9));
    plot(xp, yp, 'Parent', htpl);
end
for i = 1:4
    [xp, yp, zp, np] = infPlaneEquation(infPlanes(4 * (i - 1) + 1: 4 * (i - 1) + 4));
    plot(xp, yp, 'Parent', htpl);
    %plot('Parent', htpl, np(1,:), np(2,:), 'bx');
end
%plot(ax2, [line1(1); line1(4)], [line1(2); line1(5)]);

% wq, wc, wl, wo, wslack, wpu, wpqdot, 
weights = [1.0, 1.0, 1.0 0.0, 100000000, 0.5, 10];
safetyMarginBase = 2.0;
safetyMarginArm = 0.5;
for i=1:0
    rectangle('Parent', htpl, 'Position', [obstacles(4 * (i-1) + 1) - obstacles(4 * (i-1) + 4) obstacles(4 * (i-1) + 2) - obstacles(4 * (i-1) + 4) 2 * obstacles(4 * (i-1) + 4) 2 * obstacles(4 * (i-1) + 4)], 'Curvature', 1);
end
plot(base_spline(1, end), base_spline(2, end), 'rx');
% ht1 = hgtransform('Parent', ax2);
% posMatrix = makehgtform('translate', [start(1:2)', 0]) * makehgtform('zrotate', start(3));
% set(ht1, 'Matrix', posMatrix);
% rectangle('Parent', ht1, 'Position', [-0.5, -0.5, 1, 1]);



%% Converting obstacles and fill up

r = 0.08;
L = 0.484;

%% Debugging
base_spline1 = base_spline(:, 1:15);
param = [dt1, dt2, r, L, base_spline1(:)', o_des goal_arm_pos', weights, safetyMarginBase, safetyMarginArm, infPlanes', movingObstacles'];
%param = [dt, r, L, base_spline(:)', o_des goal_arm_pos', weights, safetyMargin];
params = repmat(param, 1, H)';
%params = repmat([dt, r, L, goal', weights, safetyMargin, obstacles'], 1, H)';
problem.all_parameters = params;

pMap = generatePMap(H);

ineq = obstacleAvoidanceSimple_1(problem.x0(1:20), param', pMap);
cost = costFunctionSimple_1(problem.x0(1:20), param', pMap);
trans = transitionFunctionSimple(10, problem.x0(1:20), params', pMap);

curState = start;
newState = start;
error = 10000;
t = 0;
base_spline1 = zeros(3, 15);
counter = 0;
pleaseDump = false;

hgmo = hgtransform('Parent', ax2, 'Matrix', eye(4));
sizeMo = 7.5;
rectangle('Parent', hgmo, 'Position', [-sizeMo, -sizeMo, sizeMo * 2, sizeMo * 2], 'Curvature', 1);

while counter < 5000
    counter = counter + 1;
    lines = ax2.findobj('type', 'Line', '-depth', 1);
    delete(lines);
    if (counter+H) > size(base_spline, 2)
        base_spline1(:, 1) = [];
        base_spline1 = [base_spline1, goal(1:3)];
    else
        base_spline1 = base_spline(:, counter+1:counter+H);
    end
    v_x = 0.05;
    v_y = -0.00;
    x = -10.5 + t * v_x;
    y = -20.0 + t * v_y;
    movingObstacles(1:7) = [x, y, -2, v_x, v_y, 0, sizeMo]';
    %movingObstacles(1:7) = [sin(t/20) * 1.5 - 1.50, -2, 0, cos(t/20) * 1.5, 0, 0, 0.5]';
    param = [dt1, dt2, r, L, base_spline1(:)', o_des goal_arm_pos', weights, safetyMarginBase, safetyMarginArm, infPlanes', movingObstacles'];


   
    moMatrix = makehgtform('translate', [movingObstacles(1), movingObstacles(2), movingObstacles(3)]);
    set(hgmo, 'Matrix', moMatrix);

    params = repmat(param, 1, H)';

    problem.all_parameters = params;
    ineq1 = obstacleAvoidanceSimple_1(problem.xinit, param', pMap);
    ineq2 = obstacleAvoidanceSimple_2(problem.xinit, param', pMap);

    [output, exitflag, info] = simplempc(problem);
    if (exitflag == -6)
        pleaseDump = true;
        %ForcesDumpProblem(problem, tag);
    elseif(pleaseDump)
        %ForcesDumpProblem(problem, tag);
        pleaseDump = false;
    end
            
        
    %curState = output.x01(1:10);
    curState = output.x02(1:10);
    fn = fieldnames(output);
    fss = zeros(15, 2);
    for i=1:numel(fn)
        if (isnumeric(output.(fn{i})))
            futureState = output.(fn{i});
            fss(i, :) = futureState(1:2);
        end
    end
    
    curU = output.x02(12:13);
    slack = output.x02(11);
    problem.xinit = output.x02;
    %problem.xinit(11:end) = zeros(10, 1);
%     problem.x0 = [output.x1;...
%         output.x2;...
%         output.x3;...
%         output.x4;...
%         output.x5];
    problem.x0 = zeros(size(problem.x0));
%     problem.x0 = [output.x01;...
%         output.x02;...
%         output.x03;...
%         output.x04;...
%         output.x05;...
%         output.x06;...
%         output.x07;...
%         output.x08;...
%         output.x09;...
%         output.x10;...
%         output.x11;...
%         output.x12;...
%         output.x13;...
%         output.x14;...
%         output.x15];
    %problem.x0 = repmat(output.x01, H, 1);
    
    
    plot(ax2, curState(1), curState(2), 'ro');
    for i = 1:15
        plot(ax2, fss(i,1), fss(i,2), 'bx');
    end
    pause(dt1/10);
    
    oldError = error;
    error = norm([curState(1:2); curState(4:end)] - [goal(1:2); goal(4:end)]);
    if (error < 0.1)
        break;
    end
    %disp(error);
    t = t + dt1;
  
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
        y = -50:0.1:50;
        x = (d - a(2) * y) / a(1);
    else
        x = -50:0.1:50;
        y = (d - a(1) * x )/ a(2);
    end
    z = zeros(size(x));
    n0 = [x(50), y(50), z(50)]';
    n1 = n0 + a';
    n = [n0, n1];
end


    

