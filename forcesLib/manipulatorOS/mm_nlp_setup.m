%% Clean up
clear; clc; close all; clearvars;
rng('shuffle');

solverName = 'MPC_OS';


disp("DEFINE THE PATH TO YOUR FORCES AND CASADI INSTALLATION");
pathForces = '/home/mspahn/develop/forces';
pathCasadi = '/home/mspahn/develop/casadi';

forcesPath = genpath(pathForces);
casadiPath = genpath(pathCasadi);
addpath(forcesPath);
addpath(casadiPath);
addpath('functionsOptimization');
addpath('dyn_model_panda');
load('functionHandle.mat')


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
model.N = 15;                                       % horizon length
nbObstacles = 0;
nbSpheres = 1;                                  % base + 5 for the arm
model.nh = nbObstacles * nbSpheres;             % number of inequality constraint functions

% [x, y, theta, q1, q2, q3, q4, q5, q6, q7, x_ee, y_ee, z_ee, #13
% u_left, u_right, q1Dot, q2Dot, q3Dot, q4Dot, q5Dot, q6Dot, q7Dot] #9
model.nvar = 3 + 7 + 3 + 2 + 7;
model.neq= 3 + 7 + 3;                               % dimension of transition function
% [dt, r, L, x_ee_des, y_ee_des, z_ee_des, obstacles...]
n_other_param = 3 + 3 + 4 * nbObstacles;    % [dt, r, L, x_des, y_des, theta_des, q_des (size : 7), q_vel_des (size : 7), obstacles(1).x, obstacles(1).y, obstacle(3), obsctacles(1).r, ...]
model.E = [eye(13, 13), zeros(13, 9)];

model.npar =  n_other_param;          % number of parameters


%% Limits for robot
q_lim_franka_up = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];
q_lim_franka_low = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973];
q_lim_franka_vel = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
q_lim_franka_torque = [87, 87, 87, 87, 12, 12, 12];
q_lim_franka_acc = [15, 7.5, 10, 12.5, 15, 20, 20, 20];
wheel_lim_vel = [5, 5];
ee_lim_low = [-inf, -inf, 0];
ee_lim_up = [inf, inf, inf];

% [x, y, theta, q u1, u2];
lower_bound = [-inf, -inf, -pi, q_lim_franka_low, ee_lim_low, -q_lim_franka_vel, -wheel_lim_vel];
upper_bound = [inf, inf, pi, q_lim_franka_up, ee_lim_up, q_lim_franka_vel, wheel_lim_vel];
model.lb = lower_bound;
model.ub = upper_bound;

%% Costs, dynamics, obstacles
model.objective = @(z, p) costFunctionOS(z, p);
model.ineq = @(z, p) obstacleAvoidanceOS(z, p);
model.eq = @(z, p) transitionFunctionOS(z, p);

model.hu = inf(nbObstacles * nbSpheres, 1);
model.hl = zeros(nbObstacles * nbSpheres, 1);



%% Initial and final conditions
% Initial condition on vehicle states

model.xinitidx = 1:22; % use this to specify on which variables initial conditions are imposed

%model.xfinal = 0; % v final=0 (standstill), heading angle final=0?
%model.xfinalidx = 6; % use this to specify on which variables final conditions are imposed

%% Define solver options
codeoptions = getOptions(solverName);
codeoptions.maxit = 250;   % Maximum number of iterations
codeoptions.printlevel = 2 ; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.timing = 0;
codeoptions.overwrite = 1;
codeoptions.mu0 = 20;
codeoptions.cleanup = 1;
codeoptions.BuildSimulinkBlock = 0;
codeoptions.nlp.lightCasadi = 0;

% codeoptions.nlp.integrator.type = 'ERK2';
% codeoptions.nlp.integrator.Ts = 0.1;
% codeoptions.nlp.integrator.nodes = 5;

FORCES_NLP(model, codeoptions);
