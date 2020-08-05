#include "mpc_forces_solver.h"

//mm_MPC_params forces_params;
//mm_MPC_output forces_output;
//mm_MPC_info forces_info;

extern "C" {
extern void mm_MPC_casadi2forces( mm_MPC_float *x,        /* primal vars                                         */
                                                 mm_MPC_float *y,        /* eq. constraint multiplers                           */
                                                 mm_MPC_float *l,        /* ineq. constraint multipliers                        */
                                                 mm_MPC_float *p,        /* parameters                                          */
                                                 mm_MPC_float *f,        /* objective function (scalar)                         */
                                                 mm_MPC_float *nabla_f,  /* gradient of objective function                      */
                                                 mm_MPC_float *c,        /* dynamics                                            */
                                                 mm_MPC_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                                 mm_MPC_float *h,        /* inequality constraints                              */
                                                 mm_MPC_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                                 mm_MPC_float *hess,     /* Hessian (column major)                              */
                                                 solver_int32_default stage,     /* stage number (0 indexed)                            */
                                                 solver_int32_default iteration /* iteration number of solver                          */);
mm_MPC_extfunc extfunc_eval = &mm_MPC_casadi2forces;
}

MpcForcesSolver::MpcForcesSolver() {}
  
MpcForcesSolver::~MpcForcesSolver() {}

void MpcForcesSolver::setupMPC(MpcProblem& mp){
  converter_.setupParams(mp);
  converter_.setForcesVariables(mp);
}

void MpcForcesSolver::solveMPC(){
  mm_MPC_params forces_params = converter_.forces_params();
  /*
  for (int i = 0; i < 20; ++i) {
    printf("xinit[%d] : %1.4f\n", i, forces_params.xinit[i]);
  }
  for (int a = 0; a < 20; ++a) {
    printf("x0[%d] : %1.4f\n", a, forces_params.x0[a]);
  }
  for (int b = 0; b < NPF; ++b) {
    printf("all_params[%d] : %1.4f\n", b, forces_params.all_parameters[b]);
  }
  */
  int exitFlag = mm_MPC_solve(&forces_params, &forces_output_, &forces_info_, stdout, extfunc_eval);
  exitFlag_ = exitFlag;
  //printf("ExitFlag : %d\n", exitFlag);
}

mm_MPC_params MpcForcesSolver::forces_params()
{
  return converter_.forces_params();
}

int MpcForcesSolver::getCurExitFlag()
{
  return exitFlag_;
}

curUArray MpcForcesSolver::getOptimalControl()
{
  curUArray optCommands;
  if (exitFlag_ == -6) 
    optCommands = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for (int i = 0; i < NU; ++i) {
    optCommands[i] = forces_output_.x02[i+NX+NS];
  }
  return optCommands;
}
  
  
