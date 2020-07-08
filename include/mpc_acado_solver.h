#ifndef MPC_SOLVER_H
#define MPC_SOLVER_H

#include "mpc_problem.h"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

class MpcSolver
{
private:
  int maximalSteps_;
  double solverTolerance_;

public:
  MpcSolver();
  ~MpcSolver();
  std::array<double, 10> solveMPC(MpcProblem&);
};

#endif /* MPC_SOLVER_H */ 
