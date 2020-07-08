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
    //printf("Iteration step %d, KKT : %1.5f\n", iter, acado_getKKT());
  }
  std::array<double, 10> optCommands;
  for (int i = 0; i < 10; ++i) {
    optCommands[i] = acadoVariables.u[i+10];
  }
  mp.slackVel(optCommands[9+10]);
  mp.slackVar(acadoVariables.x[10+11]);
  printf("Slack Variable : %1.5f\n", mp.slackVar());
  printf("Slack Velocity: %1.5f\n", mp.slackVel());
  return optCommands;
}
  
