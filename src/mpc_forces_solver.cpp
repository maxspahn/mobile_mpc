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

void MpcForcesSolver::initMPC(MpcProblem& mp){
  converter_.setupParams(mp);
  converter_.setForcesVariables(mp);
}

void MpcForcesSolver::updateMPC(MpcProblem& mp){
  converter_.updateForcesVariables(mp);
  mm_MPC_params *forces_params = converter_.forces_params();
  receideTimeHorizon(forces_params);
}

void MpcForcesSolver::solveMPC(){
  mm_MPC_params *forces_params = converter_.forces_params();
  /*
  for (int i = 0; i < 20; ++i) {
    printf("xinit[%d] : %1.4f\n", i, forces_params->xinit[i]);
  }
  for (int a = 0; a < (NX + NU + NS) * 1; ++a) {
    printf("x0[%d] : %1.4f\n", a, forces_params->x0[a]);
  }
  for (int b = 0; b < NPF; ++b) {
    printf("all_params[%d] : %1.4f\n", b, forces_params->all_parameters[b]);
  }
  */
  int exitFlag = mm_MPC_solve(forces_params, &forces_output_, &forces_info_, stdout, extfunc_eval);
  exitFlag_ = exitFlag;
  //printf("ExitFlag : %d\n", exitFlag);
}

void MpcForcesSolver::receideTimeHorizon(mm_MPC_params *forces_params)
{
    unsigned int index;
    unsigned int tot_var = NX + NS + NUF;
    for (int var = 0; var < tot_var; ++var)
    {
      forces_params->x0[0 * tot_var + var] = forces_output_.x02[var];
      forces_params->x0[1 * tot_var + var] = forces_output_.x03[var];
      forces_params->x0[2 * tot_var + var] = forces_output_.x04[var];
      forces_params->x0[3 * tot_var + var] = forces_output_.x05[var];
      forces_params->x0[4 * tot_var + var] = forces_output_.x06[var];
      forces_params->x0[5 * tot_var + var] = forces_output_.x07[var];
      forces_params->x0[6 * tot_var + var] = forces_output_.x08[var];
      forces_params->x0[7 * tot_var + var] = forces_output_.x09[var];
      forces_params->x0[8 * tot_var + var] = forces_output_.x10[var];
      forces_params->x0[9 * tot_var + var] = forces_output_.x11[var];
      forces_params->x0[10 * tot_var + var] = forces_output_.x12[var];
      forces_params->x0[11 * tot_var + var] = forces_output_.x13[var];
      forces_params->x0[12 * tot_var + var] = forces_output_.x14[var];
      forces_params->x0[13 * tot_var + var] = forces_output_.x15[var];
      forces_params->x0[14 * tot_var + var] = forces_output_.x16[var];
      forces_params->x0[15 * tot_var + var] = forces_output_.x17[var];
      forces_params->x0[16 * tot_var + var] = forces_output_.x18[var];
      forces_params->x0[17 * tot_var + var] = forces_output_.x19[var];
      forces_params->x0[18 * tot_var + var] = forces_output_.x20[var];
      //forces_params->x0[19 * tot_var + var] = forces_output_.x20[var];
    }
}
  

mm_MPC_params* MpcForcesSolver::forces_params()
{
  return converter_.forces_params();
}

double MpcForcesSolver::getSolveTime()
{
  return forces_info_.solvetime;
}

int MpcForcesSolver::getCurExitFlag()
{
  return exitFlag_;
}

int MpcForcesSolver::getNbIter()
{
  return forces_info_.it;
}

curUArray MpcForcesSolver::getOptimalControl()
{
  curUArray optCommands;
  for (int i = 0; i < NU; ++i) {
    optCommands[i] = forces_output_.x02[i+NX+NS];
  }
  if (exitFlag_ == -6) {
    for (int i = 0; i < NU; ++i) {
      optCommands[i] *= 0.9;
    }
  }
  return optCommands;
}

std::vector<curStateArray> MpcForcesSolver::getPredTraj()
{
  std::vector<curStateArray> res;
  for (int i = 2; i < TH; ++i) {
    res.push_back(curStateArray());
  }
  for (int i = 0; i < (NX + NS); ++i) {
      res[0][i] = forces_output_.x02[i];
      res[1][i] = forces_output_.x03[i];
      res[2][i] = forces_output_.x04[i];
      res[3][i] = forces_output_.x05[i];
      res[4][i] = forces_output_.x06[i];
      res[5][i] = forces_output_.x07[i];
      res[6][i] = forces_output_.x08[i];
      res[7][i] = forces_output_.x09[i];
      res[8][i] = forces_output_.x10[i];
      res[9][i] = forces_output_.x11[i];
      res[10][i] = forces_output_.x12[i];
      res[11][i] = forces_output_.x13[i];
      res[12][i] = forces_output_.x14[i];
      res[13][i] = forces_output_.x15[i];
      res[14][i] = forces_output_.x16[i];
      res[15][i] = forces_output_.x17[i];
      res[16][i] = forces_output_.x18[i];
      res[17][i] = forces_output_.x19[i];
      //res[18][i] = forces_output_.x20[i];
    }
    return res;
}
  
  
  
