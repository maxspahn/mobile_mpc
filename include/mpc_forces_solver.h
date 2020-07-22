#ifndef MPC_SOLVER_H
#define MPC_SOLVER_H

#include "forces_converter.h"
#include "mm_MPC.h"

class MpcForcesSolver
{
private:
  mm_MPC_output forces_output_;
  mm_MPC_info forces_info_;
  ForcesConverter converter_;

public:
  MpcForcesSolver();
  ~MpcForcesSolver();
  void setupMPC(MpcProblem&);
  void solveMPC();
  curUArray getOptimalControl();
  mm_MPC_params forces_params();
};

#endif /* MPC_SOLVER_H */ 
