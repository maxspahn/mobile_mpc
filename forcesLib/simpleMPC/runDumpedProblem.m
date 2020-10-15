close all;
clc;

addpath('costFunctions');
addpath('ineqFunctions');
addpath('../../dumpedCppFiles');


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
pMap.dt1 = 1;
pMap.dt2 = 2;
pMap.r = 3;
pMap.L = 4;
pMap.refPath = [5, 4 + 3 * model.N - 1];
pMap.desOrient = 5 + 3 * model.N;
pMap.desArmConfig = [pMap.desOrient + 1, pMap.desOrient + 7];
pMap.weights = [pMap.desArmConfig(2) + 1, pMap.desArmConfig(2) + 7];
pMap.safetyMargin = pMap.weights(2) + 1;
pMap.colAvoidance = collStruct;
pMap.colAvoidance.offset = pMap.safetyMargin + 1;

%% Read in dumped problems

% read .mat-files
% Exitflag -6
%fileName1 = "dumpedFiles/simplempc_O3BMONJHPKF_20201012120822786_P.mat";
%fileName1 = "dumpedFiles/simplempc_O3BMONJHPKF_20201012122957002_P.mat";
%fileName1 = "dumpedFiles/simplempc_O3BMONJHPKF_20201012131326065_P.mat";
fileName1 = "dumpedFiles/simplempc_O3BMONJHPKF_20201014180307249_P.mat";
P1 = load(fileName1);
% Exitflag 1
%fileName2 = "dumpedFiles/simplempc_O3BMONJHPKF_20201012120822889_P.mat";
%fileName2 = "dumpedFiles/simplempc_O3BMONJHPKF_20201012122957098_P.mat";
%fileName2 = "dumpedFiles/simplempc_O3BMONJHPKF_20201012131326171_P.mat";
fileName2 = "dumpedFiles/simplempc_O3BMONJHPKF_20201014180307414_P.mat";
P2 = load(fileName2);





%% Attempt to solve dumped Problems
output1 = simplempc( P1.problem );
output2 = simplempc( P2.problem);


%% Get Diff in files

diffx0 = P1.problem.x0 - P2.problem.x0;
diffxinit = P1.problem.xinit - P2.problem.xinit;
diffparam = P1.problem.all_parameters - P2.problem.all_parameters;

[~, indexMaxParamDiff] = max(abs(diffparam));
val1 = P1.problem.all_parameters(indexMaxParamDiff);
val2 = P2.problem.all_parameters(indexMaxParamDiff);

[~, indexMaxXinit] = max(abs(diffxinit));
val3 = P1.problem.xinit(indexMaxXinit);
val4 = P2.problem.xinit(indexMaxXinit);

[~, indexMaxX0] = max(abs(diffx0));
val5 = P1.problem.x0(indexMaxX0);
val6 = P2.problem.x0(indexMaxX0);

max(abs(diffx0));
max(abs(diffxinit));

%% Check constraints
z1 = P1.problem.xinit;
p1 = P1.problem.all_parameters(1:340);
ineq1 = obstacleAvoidanceSimple_1(z1, p1, pMap);
x1_2 = transitionFunctionSimple_1(z1, p1, pMap);
J1 = costFunctionSimple_1(z1, p1, pMap);


z2 = P2.problem.xinit;
p2 = P2.problem.all_parameters(1:340);
ineq2 = obstacleAvoidanceSimple_1(z2, p2, pMap);
x = transitionFunctionSimple_1(z2, p2, pMap);


%% Modify the problem

pMod = P2.problem;
%pMod.all_parameters = P2.problem.all_parameters;
%pMod.xinit = P2.problem.xinit;
%pMod.xinit(11) = 0.0;
%pMod.xinit(10) = 0.0;
%pMod.x0 = P2.problem.x0;
pMod.x0 = zeros(size(pMod.x0));
outputM = simplempc(pMod);

z = P2.problem.xinit;
p = P2.problem.all_parameters(1:340);
ineq = obstacleAvoidanceSimple_1(z, p, pMap);
min(ineq)
x_next = transitionFunctionSimple_1(z, p, pMap);
J = costFunctionSimple_1(z, p, pMap);


%% check output of problem

pAfter = P1.problem;
%pAfter.xinit = outputM.x02;
pAfter.x0 = repmat(outputM.x08, 15, 1);
simplempc(pAfter);

