#include "mpc_acado_solver.h"

ACADOworkspace acadoWorkspace;
ACADOvariables acadoVariables;

MpcAcadoSolver::MpcAcadoSolver() : 
  maximalSteps_(100),
  solverTolerance_(0.001),
  converter_()
{}
  
void MpcAcadoSolver::solveMPC(MpcProblem& mp){
  acado_timer t;
  acado_initializeSolver();
  converter_.setupParams(mp);
  converter_.setAcadoVariables(mp);
  acadoWorkspace = converter_.acadoWorkspace();
  acadoVariables = converter_.acadoVariables();
	acado_preparationStep();
  acado_feedbackStep();
  int iter = 0;
  // Currently it is not working
  while (acado_getKKT() > solverTolerance_ && iter < maximalSteps_)
  {
    acado_feedbackStep();
	  acado_preparationStep();
    iter++;
  }
  acadoWorkspace_ = acadoWorkspace;
  acadoVariables_ = acadoVariables;
}

curUArray MpcAcadoSolver::getOptimalControl()
{
  curUArray optCommands;
  for (int i = 0; i < NU; ++i) {
    optCommands[i] = acadoVariables_.u[i+10];
  }
  return optCommands;
}
  
