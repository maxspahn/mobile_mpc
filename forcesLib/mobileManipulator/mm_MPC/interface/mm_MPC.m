% mm_MPC - a fast solver generated by FORCES PRO v3.0.1
%
%   OUTPUT = mm_MPC(PARAMS) solves a multistage problem
%   subject to the parameters supplied in the following struct:
%       PARAMS.xinit - column vector of length 20
%       PARAMS.x0 - column vector of length 400
%       PARAMS.all_parameters - column vector of length 400
%
%   OUTPUT returns the values of the last iteration of the solver where
%       OUTPUT.x01 - column vector of size 20
%       OUTPUT.x02 - column vector of size 20
%       OUTPUT.x03 - column vector of size 20
%       OUTPUT.x04 - column vector of size 20
%       OUTPUT.x05 - column vector of size 20
%       OUTPUT.x06 - column vector of size 20
%       OUTPUT.x07 - column vector of size 20
%       OUTPUT.x08 - column vector of size 20
%       OUTPUT.x09 - column vector of size 20
%       OUTPUT.x10 - column vector of size 20
%       OUTPUT.x11 - column vector of size 20
%       OUTPUT.x12 - column vector of size 20
%       OUTPUT.x13 - column vector of size 20
%       OUTPUT.x14 - column vector of size 20
%       OUTPUT.x15 - column vector of size 20
%       OUTPUT.x16 - column vector of size 20
%       OUTPUT.x17 - column vector of size 20
%       OUTPUT.x18 - column vector of size 20
%       OUTPUT.x19 - column vector of size 20
%       OUTPUT.x20 - column vector of size 20
%
%   [OUTPUT, EXITFLAG] = mm_MPC(PARAMS) returns additionally
%   the integer EXITFLAG indicating the state of the solution with 
%       1 - OPTIMAL solution has been found (subject to desired accuracy)
%       0 - Timeout - maximum number of iterations reached
%      -6 - NaN or INF occured during evaluation of functions and derivatives. Please check your initial guess.
%      -7 - Method could not progress. Problem may be infeasible. Run FORCESdiagnostics on your problem to check for most common errors in the formulation.
%    -100 - License error
%
%   [OUTPUT, EXITFLAG, INFO] = mm_MPC(PARAMS) returns 
%   additional information about the last iterate:
%       INFO.it        - number of iterations that lead to this result
%       INFO.res_eq    - max. equality constraint residual
%       INFO.res_ineq  - max. inequality constraint residual
%       INFO.rsnorm    - norm of stationarity condition
%       INFO.rcompnorm    - max of all complementarity violations
%       INFO.pobj      - primal objective
%       INFO.mu        - duality measure
%       INFO.solvetime - Time needed for solve (wall clock time)
%       INFO.fevalstime - Time needed for function evaluations (wall clock time)
%
% See also COPYING
