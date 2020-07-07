#include "mpc_solver.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

MpcSolver::MpcSolver() : maximalSteps_(100), solverTolerance_(0.001) {}
  
MpcSolver::~MpcSolver() {}

std::array<double, 10> MpcSolver::solveMPC(MpcProblem& mp){
  int i;
  acado_timer t;
  acado_initializeSolver();
  mp.setAcadoVariables(acadoVariables);
	acado_preparationStep();
  acado_feedbackStep();
  int iter = 0;
  while (acado_getKKT() > solverTolerance_ && iter < maximalSteps_)
  {
    acado_feedbackStep();
	  acado_preparationStep();
    iter++;
  }
  std::array<double, 10> optCommands;
  for (int i = 0; i < 10; ++i) {
    optCommands[i] = acadoVariables.u[i+10];
  }
  return optCommands;
}
  
