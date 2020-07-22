#ifndef MPC_ACODO_SOLVER_H
#define MPC_ACODO_SOLVER_H

#include "acado_converter.h"

class MpcAcadoSolver
{
private:
  int maximalSteps_;
  double solverTolerance_;
  AcadoConverter converter_;
  ACADOworkspace acadoWorkspace_;
  ACADOvariables acadoVariables_;

public:
  MpcAcadoSolver();
  void solveMPC(MpcProblem&);
  curUArray getOptimalControl();
};

#endif /* MPC_ACODO_SOLVER_H */ 
