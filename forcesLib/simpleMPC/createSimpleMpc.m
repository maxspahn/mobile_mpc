%% Clean up
clear; clc; close all; clearvars;
rng('shuffle');

solverName = 'simplempc';

disp("DEFINE THE PATH TO YOUR FORCES AND CASADI INSTALLATION");
pathForces = '/home/mspahn/develop/forces';
pathCasadi = '/home/mspahn/develop/casadi';

forcesPath = genpath(pathForces);
casadiPath = genpath(pathCasadi);
addpath(forcesPath);
addpath(casadiPath);

addpath('../distanceFunctions');


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

%% Problem dimensions
model.N = 15;                                           % horizon length
collStruct.nbObstacles = 0;
collStruct.nbMovingObstacles = 5;
collStruct.dimMovingObstacle = 7;
collStruct.nbSpheres = 4;
collStruct.nbSelfCollision = 0;
collStruct.nbPlanes = 0;
collStruct.nbInfPlanesEachSphere = 15;
collStruct.dimPlane = 9;
collStruct.dimInfPlane = 4;
collStruct.dimObstacle = 4;

pMap = struct();
pMap.dt = 1;
pMap.r = 2;
pMap.L = 3;
pMap.refPath = [4, 4 + 3 * model.N - 1];
pMap.desOrient = 4 + 3 * model.N;
pMap.desArmConfig = [pMap.desOrient + 1, pMap.desOrient + 7];
pMap.weights = [pMap.desArmConfig(2) + 1, pMap.desArmConfig(2) + 7];
pMap.safetyMargin = pMap.weights(2) + 1;
pMap.colAvoidance = collStruct;
pMap.colAvoidance.offset = pMap.safetyMargin + 1;

nbInfPlanes = collStruct.nbInfPlanesEachSphere * collStruct.nbSpheres;

nbInequalities = (collStruct.nbObstacles + collStruct.nbPlanes + collStruct.nbInfPlanesEachSphere + collStruct.nbMovingObstacles) * collStruct.nbSpheres +...
    collStruct.nbSelfCollision; 
model.nh = nbInequalities;   % number of inequality constraint functions
n_other_param = 3 +... % timeStep, r, L (1, 2, 3)
    3 *  model.N + ... % Reference path (4, ..., 4 + 3 * N) [x, y, orientation]
    1 + ... % desired orientation
    7 + ... % desired arm configuration
    7 + ... % weights
    1 + ... % safety Margin
    collStruct.dimObstacle * collStruct.nbObstacles +...
    collStruct.dimPlane * collStruct.nbPlanes + ...
    collStruct.dimInfPlane * nbInfPlanes + ...
    collStruct.dimMovingObstacle * collStruct.nbMovingObstacles;


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
wheel_lim_vel = 2;
wheel_lim_acc = 25;
slack_lim_low = 0;
slack_lim_up = inf;
velocity_safety = 1.0;
wheel_lim_back = -2;

lower_bound = [-inf, -inf, -inf, q_lim_franka_low, slack_lim_low, wheel_lim_back, wheel_lim_back, -q_lim_franka_vel];
upper_bound = [inf, inf, inf, q_lim_franka_up, slack_lim_up, wheel_lim_vel, wheel_lim_vel, q_lim_franka_vel];

model.lb = velocity_safety * lower_bound;
model.ub = velocity_safety * upper_bound;

%% Costs, dynamics, obstacles
for i=1:model.N
    %% Objective function
    model.objective{i} = @(z, p) costFunctionSimple(z, p, pMap, i);
    model.ineq{i} = @(z, p) obstacleAvoidanceSimple(z, p, pMap, i);
    %% Upper/lower bounds For road boundaries
    model.hu{i} = inf(nbInequalities, 1);
    model.hl{i} = zeros(nbInequalities, 1);
end

% Transition Function
model.eq = @(z, p) transitionFunctionSimple(z, p, pMap);

%% Initial and final conditions

model.xinitidx = 1:20; % use this to specify on which variables initial conditions are imposed

%model.xfinal = 0; % v final=0 (standstill), heading angle final=0?
%model.xfinalidx = 6; % use this to specify on which variables final conditions are imposed

%% Define solver options
codeoptions = getOptions(solverName);
codeoptions.maxit = 200;   % Maximum number of iterations
codeoptions.printlevel = 1; % Use printlevel = 2 to print progress (but not for timings)
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
