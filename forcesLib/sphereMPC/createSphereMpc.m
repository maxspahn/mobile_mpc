%% Clean up
clear; clc; close all; clearvars;
rng('shuffle');

nbSpheres = 17;
solverName = ['sphere', num2str(nbSpheres)];

disp("DEFINE THE PATH TO YOUR FORCES AND CASADI INSTALLATION");
pathForces = '/home/mspahn/develop/forces';
pathCasadi = '/home/mspahn/develop/casadi';

forcesPath = genpath(pathForces);
casadiPath = genpath(pathCasadi);
addpath(forcesPath);
addpath(casadiPath);

addpath('../distanceFunctions');
addpath('../optimizationFunctions/costFunctions');
addpath('../optimizationFunctions/ineqFunctions');
addpath('../optimizationFunctions/dynamics');



% Turn off warnings for having a too recent Gcc compiler and some
% SoapService
warning('off', 'MATLAB:mex:GccVersion_link');
warning('off', 'MATLAB:webservices:WSDLDeprecationWarning');

%% Delete previous Solver
% Forces does not always code changes and might reuse the previous solution
try
    FORCEScleanup(solverName,'all');
catch
end

try
    rmdir('@FORCESproWS','s')
catch
end
try
    rmdir(solverName,'s')
catch
end


%% Problem dimensions
model.N = 15;                                           % horizon length

pMap = generatePMapSpheres(model.N, nbSpheres);

nbInequalities = pMap.nbStaticSpheres * 4 + pMap.nbMovingObstacles * 4; 
model.nh = nbInequalities;   % number of inequality constraint functions
n_other_param = pMap.staticSpheres(2);


model.nvar = 3 + 7 + 1 + 2 + 7;                     % number of variables [x, y, theta, q (size : 7), slack, u1, u2, q_dot (size : 7)]
model.neq= 3 + 7;                               % dimension of transition function
model.E = [eye(10, 10), zeros(10, 10)];

model.npar =  n_other_param;          % number of parameters


%% Limits for robot
q_lim_franka_up = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];
q_lim_franka_low = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973];
q_lim_franka_vel = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
q_lim_franka_torque = [87, 87, 87, 87, 12, 12, 12];
q_lim_franka_acc = [15, 7.5, 10, 12.5, 15, 20, 20];
wheel_lim_vel = 5; % physical limit 25
wheel_lim_back = -5;
wheel_lim_acc = 25;
slack_lim_low = 0;
slack_lim_up = inf;
velocity_safety = 1.0;

lower_bound = [-inf, -inf, -inf, q_lim_franka_low, slack_lim_low, wheel_lim_back, wheel_lim_back, -q_lim_franka_vel];
upper_bound = [inf, inf, inf, q_lim_franka_up, slack_lim_up, wheel_lim_vel, wheel_lim_vel, q_lim_franka_vel];

model.lb = velocity_safety * lower_bound;
model.ub = velocity_safety * upper_bound;

model.objective{1} =  @(z, p) costFunctionSimple_1(z, p, pMap);
model.objective{2} =  @(z, p) costFunctionSimple_2(z, p, pMap);
model.objective{3} =  @(z, p) costFunctionSimple_3(z, p, pMap);
model.objective{4} =  @(z, p) costFunctionSimple_4(z, p, pMap);
model.objective{5} =  @(z, p) costFunctionSimple_5(z, p, pMap);
model.objective{6} =  @(z, p) costFunctionSimple_6(z, p, pMap);
model.objective{7} =  @(z, p) costFunctionSimple_7(z, p, pMap);
model.objective{8} =  @(z, p) costFunctionSimple_8(z, p, pMap);
model.objective{9} =  @(z, p) costFunctionSimple_9(z, p, pMap);
model.objective{10} = @(z, p) costFunctionSimple_10(z, p, pMap);
model.objective{11} = @(z, p) costFunctionSimple_11(z, p, pMap);
model.objective{12} = @(z, p) costFunctionSimple_12(z, p, pMap);
model.objective{13} = @(z, p) costFunctionSimple_13(z, p, pMap);
model.objective{14} = @(z, p) costFunctionSimple_14(z, p, pMap);
model.objective{15} = @(z, p) costFunctionSimple_15(z, p, pMap);

model.ineq{1} =  @(z, p) obstacleAvoidanceSphere_1(z, p, pMap);
model.ineq{2} =  @(z, p) obstacleAvoidanceSphere_2(z, p, pMap);
model.ineq{3} =  @(z, p) obstacleAvoidanceSphere_3(z, p, pMap);
model.ineq{4} =  @(z, p) obstacleAvoidanceSphere_4(z, p, pMap);
model.ineq{5} =  @(z, p) obstacleAvoidanceSphere_5(z, p, pMap);
model.ineq{6} =  @(z, p) obstacleAvoidanceSphere_6(z, p, pMap);
model.ineq{7} =  @(z, p) obstacleAvoidanceSphere_7(z, p, pMap);
model.ineq{8} =  @(z, p) obstacleAvoidanceSphere_8(z, p, pMap);
model.ineq{9} =  @(z, p) obstacleAvoidanceSphere_9(z, p, pMap);
model.ineq{10} = @(z, p) obstacleAvoidanceSphere_10(z, p, pMap);
model.ineq{11} = @(z, p) obstacleAvoidanceSphere_11(z, p, pMap);
model.ineq{12} = @(z, p) obstacleAvoidanceSphere_12(z, p, pMap);
model.ineq{13} = @(z, p) obstacleAvoidanceSphere_13(z, p, pMap);
model.ineq{14} = @(z, p) obstacleAvoidanceSphere_14(z, p, pMap);
model.ineq{15} = @(z, p) obstacleAvoidanceSphere_15(z, p, pMap);

%% Costs, dynamics, obstacles
for i=1:model.N
    % Upper/lower bounds For road boundaries
    model.hu{i} = inf(nbInequalities, 1);
    model.hl{i} = zeros(nbInequalities, 1);
    % Transition Function
    if (i < 5)
        model.eq{i} = @(z, p) transitionFunctionSimple_1(z, p, pMap);
    elseif (i < model.N)
        model.eq{i} = @(z, p) transitionFunctionSimple_2(z, p, pMap);
    end
end


%% Initial and final conditions

model.xinitidx = [1:10, 12:20]; % use this to specify on which variables initial conditions are imposed


%% Define solver options
codeoptions = getOptions(solverName);
codeoptions.maxit = 200;   % Maximum number of iterations
codeoptions.printlevel = 0; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 3;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.timing = 1;
codeoptions.overwrite = 1;
codeoptions.mu0 = 20;
codeoptions.cleanup = 1;
codeoptions.BuildSimulinkBlock = 0;
codeoptions.nlp.lightCasadi = 0;
codeoptions.threadSafeStorage = true;
codeoptions.nlp.BarrStrat = 'loqo';
codeoptions.nlp.checkFunctions = 1;
%codeoptions.accuracy.ineq = 0.1;


% Dumping the problem
% codeoptions.dump_formulation = 1;
% [stages, codeoptions, formulation] = FORCES_NLP(model, codeoptions);
% tag = ForcesDumpFormulation(formulation, codeoptions);

% codeoptions.nlp.integrator.type = 'ERK2';
% codeoptions.nlp.integrator.Ts = 0.1;
% codeoptions.nlp.integrator.nodes = 5;

FORCES_NLP(model, codeoptions);
