%% Clean up
clear; clc; close all; clearvars;
rng('shuffle');

solverName = 'mm_MPC';
dynamics = 'simple';
%dynamics = 'torques';
%dynamics = 'acc';


disp("DEFINE THE PATH TO YOUR FORCES AND CASADI INSTALLATION");
pathForces = '/home/mspahn/develop/forces';
pathCasadi = '/home/mspahn/develop/casadi';

forcesPath = genpath(pathForces);
casadiPath = genpath(pathCasadi);
addpath(forcesPath);
addpath(casadiPath);
addpath('functionsOptimization');
addpath('dyn_model_panda');
addpath('distanceFunctions');

% Turn off warnings for having a too recent Gcc compiler and some
% SoapService
warning('off', 'MATLAB:mex:GccVersion_link');
warning('off', 'MATLAB:webservices:WSDLDeprecationWarning');

%% Delete previous Solver
% Forces does not always code changes and might reuse the previous solution
try
FORCEScleanup('solverName','all');
catch
end

try
    rmdir('@FORCESproWS','s')
catch
end
try
    rmdir('solverName','s')
catch
end
% 
%% Some utility functions
deg2rad = @(deg) deg/180*pi; % convert degrees into radians
rad2deg = @(rad) rad/pi*180; % convert radians into degrees

%% Problem dimensions
model.N = 20;                                       % horizon length
nbObstacles = 5;
nbSpheres = 6;                                          % base + 5 for the arm
nbSelfCollision = 0;
nbPlanes = 8;
nbInequalities = (nbObstacles + nbPlanes) * nbSpheres + nbSelfCollision; 
model.nh = nbInequalities;   % number of inequality constraint functions

if strcmp(dynamics, 'torques')
    model.nvar = 3 + 7 + 7 + 2 + 7;                     % number of variables [x, y, theta, q (size : 7), q_dot (size : 7), u1, u2, tau (size : 7)]
    model.neq= 3 + 7 + 7;                               % dimension of transition function
    n_other_param = 3 + 3 + 7 + 6 + 4 * nbObstacles;    % [dt, r, L, x_des, y_des, theta_des, q_des (size : 7), q_vel_des (size : 7), obstacles(1).x, obstacles(1).y, obstacle(3), obsctacles(1).r, ...]
    model.E = [eye(17, 17), zeros(17, 9)];
elseif strcmp(dynamics, 'acc')
    model.nvar = 3 + 2 + 7 + 7 + 1 + 2 + 7;             % number of variables [x, y, theta, u1, u2, q (size : 7), q_dot (size : 7), slack, u1dot, u2dot, q_dotdot (size : 7)]
    model.neq= 3 + 2 + 7 + 7;                           % dimension of transition function
    n_other_param = 3 + 3 + 7 + 8 + 4 * nbObstacles;    % [dt, r, L, x_des, y_des, theta_des, q_des (size : 7), weights, obstacles(1).x, obstacles(1).y, obstacle(3), obsctacles(1).r, ...]
    model.E = [eye(19, 19), zeros(19, 10)];
elseif strcmp(dynamics, 'simple')
    model.nvar = 3 + 7 + 1 + 2 + 7;                     % number of variables [x, y, theta, q (size : 7), slack, u1, u2, q_dot (size : 7)]
    model.neq= 3 + 7;                               % dimension of transition function
    n_other_param = 3 + 3 + 7 + 6 + 1 + 4 * nbObstacles + 9 * nbPlanes;    % [dt, r, L, x_des, y_des, theta_des, q_des (size : 7), weights, obstacles(1).x, obstacles(1).y, obstacle(3), obsctacles(1).r, ...]
    model.E = [eye(10, 10), zeros(10, 10)];
end
model.npar =  n_other_param;          % number of parameters


%% Limits for robot
q_lim_franka_up = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];
q_lim_franka_low = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973];
q_lim_franka_vel = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
q_lim_franka_torque = [87, 87, 87, 87, 12, 12, 12];
q_lim_franka_acc = [15, 7.5, 10, 12.5, 15, 20, 20];
wheel_lim_vel = 5;
wheel_lim_acc = 25;
slack_lim_low = 0;
slack_lim_up = inf;
velocity_safety = 0.8;

% [x, y, theta, q u1, u2];
if strcmp(dynamics, 'torques')
    lower_bound = [-inf, -inf, -inf, q_lim_franka_low, -q_lim_franka_vel, -wheel_lim_vel, -wheel_lim_vel, -q_lim_franka_torque];
    upper_bound = [inf, inf, inf, q_lim_franka_up, q_lim_franka_vel, wheel_lim_vel, wheel_lim_vel, q_lim_franka_torque];
elseif strcmp(dynamics, 'acc')
    lower_bound = [-inf, -inf, -inf, -wheel_lim_vel, -wheel_lim_vel, q_lim_franka_low, -q_lim_franka_vel, slack_lim_low, -wheel_lim_acc, -wheel_lim_acc, -q_lim_franka_acc];
    upper_bound = [inf, inf, inf, wheel_lim_vel, wheel_lim_vel, q_lim_franka_up, q_lim_franka_vel, slack_lim_up, wheel_lim_acc, wheel_lim_acc, q_lim_franka_acc];
elseif strcmp(dynamics, 'simple')
    lower_bound = [-inf, -inf, -inf, q_lim_franka_low, slack_lim_low, -wheel_lim_vel, -wheel_lim_vel, -q_lim_franka_vel];
    upper_bound = [inf, inf, inf, q_lim_franka_up, slack_lim_up, wheel_lim_vel, wheel_lim_vel, q_lim_franka_vel];
end
model.lb = velocity_safety * lower_bound;
model.ub = velocity_safety * upper_bound;

%% Costs, dynamics, obstacles
for i=1:model.N
    %% Objective function
    if strcmp(dynamics, 'torques')
        model.objective{i} = @(z, p) costFunctionTorques(z, p);
        model.ineq{i} = @(z, p) obstacleAvoidanceTorques(z, p);
    elseif strcmp(dynamics, 'acc')
        model.objective{i} = @(z, p) costFunctionAcc(z, p);
        model.ineq{i} = @(z, p) obstacleAvoidanceAcc(z, p);
        %model.continous_dynamics = @continousDynamicsSimple;
    elseif strcmp(dynamics, 'simple')
        model.objective{i} = @(z, p) costFunctionSimple(z, p);
        model.ineq{i} = @(z, p) obstacleAvoidanceSimple(z, p);
        %model.continous_dynamics = @continousDynamicsSimple;
    end
    %% Upper/lower bounds For road boundaries
    model.hu{i} = inf(nbInequalities, 1);
    model.hl{i} = zeros(nbInequalities, 1);
end
if strcmp(dynamics, 'torques')
%     model.objective = @(z, p) costFunctionTorques(z, p);
%     model.ineq = @(z, p) obstacleAvoidanceTorques(z, p);
    model.eq = @(z, p) transitionFunctionTorques(z, p);
elseif strcmp(dynamics, 'acc')
%     model.objective = @(z, p) costFunctionSimple(z, p);
%     model.ineq = @(z, p) obstacleAvoidanceSimple(z, p);
    model.eq = @(z, p) transitionFunctionAcc(z, p);
    %model.continous_dynamics = @continousDynamicsSimple;
elseif strcmp(dynamics, 'simple')
%     model.objective = @(z, p) costFunctionSimple(z, p);
%     model.ineq = @(z, p) obstacleAvoidanceSimple(z, p);
    model.eq = @(z, p) transitionFunctionSimple(z, p);
    %model.continous_dynamics = @continousDynamicsSimple;
end

%model.hu = inf(nbObstacles * nbSpheres, 1);
%model.hl = zeros(nbObstacles * nbSpheres, 1);



%% Initial and final conditions
% Initial condition on vehicle states

if strcmp(dynamics, 'torques')
    model.xinitidx = 1:26; % use this to specify on which variables initial conditions are imposed
elseif strcmp(dynamics, 'acc')
    model.xinitidx = 1:29; % use this to specify on which variables initial conditions are imposed
elseif strcmp(dynamics, 'simple')
    model.xinitidx = 1:20; % use this to specify on which variables initial conditions are imposed
end
%model.xfinal = 0; % v final=0 (standstill), heading angle final=0?
%model.xfinalidx = 6; % use this to specify on which variables final conditions are imposed

%% Define solver options
codeoptions = getOptions(solverName);
codeoptions.maxit = 500;   % Maximum number of iterations
codeoptions.printlevel = 2 ; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.timing = 1;
codeoptions.overwrite = 1;
codeoptions.mu0 = 20;
codeoptions.cleanup = 1;
codeoptions.BuildSimulinkBlock = 0;
codeoptions.nlp.lightCasadi = 0;
codeoptions.threadSafeStorage = true;

% codeoptions.nlp.integrator.type = 'ERK2';
% codeoptions.nlp.integrator.Ts = 0.1;
% codeoptions.nlp.integrator.nodes = 5;

FORCES_NLP(model, codeoptions);
