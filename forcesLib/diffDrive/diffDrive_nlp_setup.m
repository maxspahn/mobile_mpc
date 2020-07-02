%% Clean up
clear; clc; close all; clearvars;
rng('shuffle');

solverName = 'diffDrive_MPC';

forcesPath = genpath('/home/mspahn/develop/forces');
casadiPath = genpath('/home/mspahn/develop/casadi');
addpath(forcesPath);
addpath(casadiPath);
addpath('functionsOptimization');

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
model.N = 15;            % horizon length
model.nvar = 5;          % number of variables [x, y, theta, u1, u2]
model.neq= 3; 
nbObstacles = 1;
nbSpheres = 1;
model.nh = nbObstacles * nbSpheres;           % number of inequality constraint functions
n_other_param = 3 + 3 + 4 * nbObstacles;       % [dt, r, L, x_des, y_des, theta_des, obstacles(1).x, obstacles(1).y, obstacle(3), obsctacles(1).r, ...]

model.npar =  n_other_param;          % number of parameters

%% Limits for robot
% [x, y, theta, u1, u2];
lower_bound = [0, 0, -pi, -100, -100];
upper_bound = [15, 15, pi, 100, 100];
model.lb = lower_bound;
model.ub = upper_bound;

%%
model.objective = @(z, p) costFunction(z, p);
model.ineq = @(z, p) obstacleAvoidance(z, p);
model.hu = inf(nbObstacles * nbSpheres, 1);
model.hl = zeros(nbObstacles * nbSpheres, 1);

%% Dynamics, i.e. equality constraints 
%model.objective = @(z, p) objective_scenario_try(z, p);
model.eq = @(z, p) transitionFunction(z, p);

model.E = [eye(3, 3), zeros(3, 2)];

%% Initial and final conditions
% Initial condition on vehicle states

model.xinitidx = 1:5; % use this to specify on which variables initial conditions are imposed
%model.xfinal = 0; % v final=0 (standstill), heading angle final=0?
%model.xfinalidx = 6; % use this to specify on which variables final conditions are imposed

%% Define solver options
codeoptions = getOptions(solverName);
codeoptions.maxit = 250;   % Maximum number of iterations
codeoptions.printlevel = 1 ; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.timing = 0;
codeoptions.overwrite = 1;
codeoptions.mu0 = 20;
codeoptions.cleanup = 1;
codeoptions.BuildSimulinkBlock = 0;
%codeoptions.nlp.lightCasadi = 1;

FORCES_NLP(model, codeoptions);
