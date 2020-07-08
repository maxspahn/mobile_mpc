#ifndef MPC_SOLVER_H
#define MPC_SOLVER_H

#include "mpc_problem.h"
#include "mm_MPC.h"


class MpcForcesSolver
{
private:

public:
  MpcForcesSolver();
  ~MpcForcesSolver();
  std::array<double, 10> solveMPC(MpcProblem&);
};

#endif /* MPC_SOLVER_H */ 
