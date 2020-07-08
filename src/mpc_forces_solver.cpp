#include "mpc_forces_solver.h"

mm_MPC_params forces_params;
mm_MPC_output forces_output;
mm_MPC_info forces_info;

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

std::array<double, 10> MpcForcesSolver::solveMPC(MpcProblem& mp){
  int i;
  mp.setForcesVariables(forces_params);
  printf("at exit : %1.5f\n", forces_params.all_parameters[0]);
  int exitFlag = mm_MPC_solve(&forces_params, &forces_output, &forces_info, stdout, extfunc_eval);
  std::array<double, 10> optCommands;
  printf("ExitFlag : %d\n", exitFlag);
  for (int i = 0; i < 10; ++i) {
    optCommands[i] = forces_output.x02[i+10];
  }
  /*
  mp.slackVel(optCommands[9+10]);
  mp.slackVar(acadoVariables.x[10+11]);
  printf("Slack Variable : %1.5f\n", mp.slackVar());
  printf("Slack Velocity: %1.5f\n", mp.slackVel());
  */
  return optCommands;
}
  
