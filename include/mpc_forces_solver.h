#ifndef MPC_FORCES_SOLVER_H
#define MPC_FORCES_SOLVER_H

#include "forces_converter.h"
#include "mm_MPC.h"

class MpcForcesSolver
{
private:
  mm_MPC_output forces_output_;
  mm_MPC_info forces_info_;
  ForcesConverter converter_;
  int exitFlag_;

public:
  MpcForcesSolver();
  ~MpcForcesSolver();
  void initMPC(MpcProblem&);
  void updateMPC(MpcProblem&);
  void solveMPC();
  void receideTimeHorizon(mm_MPC_params*);
  curUArray getOptimalControl();
  std::vector<curStateArray> getPredTraj();
  int getCurExitFlag();
  double getSolveTime();
  int getNbIter();
  mm_MPC_params* forces_params();
};

#endif /* MPC_FORCES_SOLVER_H */ 
