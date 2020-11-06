close all;
clc;

addpath('costFunctions');
addpath('ineqFunctions');
addpath('../../dumpedCppFiles');


%% Problem dimensions
pMap = generatePMap(15);

%% Read in dumped problems

% read .csv-files
P3.problem = readCppDumpFile("../../dumpedCppFiles/dump_20201015114327_0.csv");



%% Attempt to solve dumped Problems

output3 = simplempc(P3.problem);


%% Check constraints
z1 = P3.problem.xinit;
p1 = P3.problem.all_parameters(1:341);
ineq1 = obstacleAvoidanceSimple_1(z1, p1, pMap);


z5 = output3.x05;

ineq1  = obstacleAvoidanceSimple_1(output.x01, p1, pMap);
ineq2  = obstacleAvoidanceSimple_2(output.x02, p1, pMap);
ineq3  = obstacleAvoidanceSimple_3(output.x03, p1, pMap);
ineq4  = obstacleAvoidanceSimple_4(output.x04, p1, pMap);
ineq5  = obstacleAvoidanceSimple_5(output.x05, p1, pMap);
ineq6  = obstacleAvoidanceSimple_6(output.x06, p1, pMap);
ineq7  = obstacleAvoidanceSimple_7(output.x07, p1, pMap);
ineq8  = obstacleAvoidanceSimple_8(output.x08, p1, pMap);
ineq9  = obstacleAvoidanceSimple_9(output.x09, p1, pMap);
ineq10 = obstacleAvoidanceSimple_10(output.x10, p1, pMap);
ineq11 = obstacleAvoidanceSimple_11(output.x11, p1, pMap);
ineq12 = obstacleAvoidanceSimple_12(output.x12, p1, pMap);
ineq13 = obstacleAvoidanceSimple_13(output.x13, p1, pMap);

%% Modify the problem

pMod = P3.problem;
%pMod.all_parameters = P2.problem.all_parameters;
%pMod.xinit = P2.problem.xinit;
%pMod.xinit(11) = 0.0;
%pMod.xinit(10) = 0.0;
%pMod.x0 = P2.problem.x0;
pMod.xinit(11) = 1.0;
outputM = simplempc(pMod);

%% Get Diff in files

% diffx0 = P1.problem.x0 - P2.problem.x0;
% diffxinit = P1.problem.xinit - P2.problem.xinit;
% diffparam = P1.problem.all_parameters - P2.problem.all_parameters;
% 
% [~, indexMaxParamDiff] = max(abs(diffparam));
% val1 = P1.problem.all_parameters(indexMaxParamDiff);
% val2 = P2.problem.all_parameters(indexMaxParamDiff);
% 
% [~, indexMaxXinit] = max(abs(diffxinit));
% val3 = P1.problem.xinit(indexMaxXinit);
% val4 = P2.problem.xinit(indexMaxXinit);
% 
% [~, indexMaxX0] = max(abs(diffx0));
% val5 = P1.problem.x0(indexMaxX0);
% val6 = P2.problem.x0(indexMaxX0);
% 
% max(abs(diffx0));
% max(abs(diffxinit));


%% Modify the problem

pMod = P3.problem;
%pMod.all_parameters = P2.problem.all_parameters;
%pMod.xinit = P2.problem.xinit;
%pMod.xinit(11) = 0.0;
%pMod.xinit(10) = 0.0;
%pMod.x0 = P2.problem.x0;
pMod.x0 = zeros(size(pMod.x0));
outputM = simplempc(pMod);

z = P3.problem.xinit;
p = P3.problem.all_parameters(1:341);
ineq = obstacleAvoidanceSimple_1(z, p, pMap);
min(ineq)
x_next = transitionFunctionSimple_1(z, p, pMap);
J = costFunctionSimple_1(z, p, pMap);


%% check output of problem

pAfter = P3.problem;
%pAfter.xinit = outputM.x02;
pAfter.x0 = repmat(outputM.x08, 15, 1);
simplempc(pAfter);

