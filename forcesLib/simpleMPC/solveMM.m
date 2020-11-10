close all;
clear;
clc;

addpath('../');
addCustomPaths();

addpath('../distanceFunctions');
addpath('../optimizationFunctions/costFunctions');
addpath('../optimizationFunctions/ineqFunctions');


%% Creating the figure

fig1 = figure('name', "Mpc solving for mobile manipulator");
set(fig1, 'WindowStyle', 'docked');
ax1 = axes('Parent', fig1, 'xlim', [-50, 50], 'ylim', [-50, 50]);
hold(ax1, 'on');


%% Parameters
H = 15; %needs to be consistent with generation

dt1 = 1.0;
dt2 = 2.0;

% geometric parameters
r = 0.08;
L = 0.484;

% wq, wc, wl, wo, wslack, wpu, wpqdot, 
weights = [1.0, 1.0, 1.0 0.0, 100000000, 0.5, 10];
safetyMarginBase = 2.0;
safetyMarginArm = 0.5;

% goal definition
base_pos = [0; 0; 0.0];
arm_pos = [0; 0; 0; -1; 0; 1; 0];
u_start = zeros(9, 1);
start = [base_pos; arm_pos];
x_spline = 1 * (0.0:1.5:30.0);
y_spline = -1 * (0.0:1.5:30.0);
o_spline = ones(size(x_spline)) * atan(y_spline(3)/x_spline(3));
base_spline = [x_spline; y_spline; o_spline];
o_des = 0.0;
goal = base_spline(:,end);
goal_arm_pos = [1; 0; -1.5; -1.5; 0.5; 1; 0.2];
goal = [goal; goal_arm_pos];


%% Set up obstacles

pMap = generatePMap(H);


% generating empty obstacles
defaultObstacle = [0, 0, 0, -100];
defaultMovingObstacle = [0, 0, 0, 0, 0, 0, -100];
defaultInfPlane = [0, 0, 1, -3];

nbObstacles = 0;
obstacles = repmat(defaultObstacle', nbObstacles, 1);
movingObstacles = repmat(defaultMovingObstacle', pMap.nbMovingObstacles, 1);
infPlanes = repmat(defaultInfPlane', pMap.nbInfPlanes, 1);

% changing the obstacles
% obstacles(1:4) = [1; -3; 2.1; 1];
infPlanes(1:4) = [-1, -1, 0, -8]';
infPlanes(5:8) = [-1, 0, 0, -10]';
infPlanes(9:12) = [0, 1, 0, -40.5]';
infPlanes(13:16) = [1, 1, 0, -16]';

% Plotting obstacles
htpl = hgtransform('Parent', ax1, 'Matrix', eye(4));
for i = 1:4
    [xp, yp, zp, np] = infPlaneEquation(infPlanes(4 * (i - 1) + 1: 4 * (i - 1) + 4));
    plot(xp, yp, 'Parent', htpl);
end

for i=1:0
    rectangle('Parent', htpl, 'Position', [obstacles(4 * (i-1) + 1) - obstacles(4 * (i-1) + 4) obstacles(4 * (i-1) + 2) - obstacles(4 * (i-1) + 4) 2 * obstacles(4 * (i-1) + 4) 2 * obstacles(4 * (i-1) + 4)], 'Curvature', 1);
end
% Plotting goal
plot(base_spline(1, end), base_spline(2, end), 'rx', 'Parent', htpl);


%% Setting up MPC parameters
problem.xinit = [start; u_start];
problem.x0 = repmat([start; u_start; 0], H, 1);
base_spline1 = base_spline(:, 1:H);
param = [dt1, dt2, r, L, base_spline1(:)', o_des goal_arm_pos', weights, safetyMarginBase, safetyMarginArm, infPlanes', movingObstacles'];

params = repmat(param, 1, H)';
problem.all_parameters = params;


ineq = obstacleAvoidanceSimple_1(problem.x0(1:20), param', pMap);
cost = costFunctionSimple_1(problem.x0(1:20), param', pMap);
trans = transitionFunctionSimple(10, problem.x0(1:20), params', pMap);

%% setup loop values

curState = start;
newState = start;
error = 10000;
t = 0;
counter = 0;
pleaseDump = false;

%moving obstacle velocity
v_x = 0.2;
v_y = -0.00;

hgmo = hgtransform('Parent', ax1, 'Matrix', eye(4));
sizeMo = 7.5;
rectangle('Parent', hgmo, 'Position', [-sizeMo, -sizeMo, sizeMo * 2, sizeMo * 2], 'Curvature', 1);

while t < 100
    counter = counter + 1;
    % clean up old predictions and positions
    lines = ax1.findobj('type', 'Line', '-depth', 1);
    delete(lines);
    
    if (counter+H) > size(base_spline, 2)
        base_spline1(:, 1) = [];
        base_spline1 = [base_spline1, goal(1:3)];
    else
        base_spline1 = base_spline(:, counter+1:counter+H);
    end
    
    % computing new position of moving obstacle
    x = -10.5 + t * v_x;
    y = -20.0 + t * v_y;
    movingObstacles(1:7) = [x, y, -2, v_x, v_y, 0, sizeMo]';
    %movingObstacles(1:7) = [sin(t/20) * 1.5 - 1.50, -2, 0, cos(t/20) * 1.5, 0, 0, 0.5]';    
    moMatrix = makehgtform('translate', [movingObstacles(1), movingObstacles(2), movingObstacles(3)]);
    set(hgmo, 'Matrix', moMatrix);

    
    
    param = [dt1, dt2, r, L, base_spline1(:)', o_des goal_arm_pos', weights, safetyMarginBase, safetyMarginArm, infPlanes', movingObstacles'];
    params = repmat(param, 1, H)';
    problem.all_parameters = params;
    

    [output, exitflag, info] = simplempc(problem);
    fprintf("Exitflag %d Time %1.5f itartions %d\n", exitflag, info.solvetime, info.it);
    
    % Debugging via dumped files
    % Quickly fills up the workspace
    if (exitflag == -6)
        pleaseDump = true;
        %ForcesDumpProblem(problem, tag);
    elseif(pleaseDump)
        %ForcesDumpProblem(problem, tag);
        pleaseDump = false;
    end
            
        
    curState = output.x02(1:10);
    
    % Extracting future base states
    fn = fieldnames(output);
    fss = zeros(15, 2);
    for i=1:numel(fn)
        if (isnumeric(output.(fn{i})))
            futureState = output.(fn{i});
            fss(i, :) = futureState(1:2);
        end
    end
    
    plot(ax1, curState(1), curState(2), 'ro');
    for i = 1:15
        plot(ax1, fss(i,1), fss(i,2), 'bx');
    end
    
    curU = output.x02(12:13);
    slack = output.x02(11);
    problem.xinit = [output.x02(1:10); output.x02(12:end)];

    % Receide time horizon
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
        output.x15];
    
    pause(dt1/10);
    
    oldError = error;
    error = norm([curState(1:2); curState(4:end)] - [goal(1:2); goal(4:end)]);
    if (error < 0.1)
        break;
    end
    t = t + dt1;
  
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


    

