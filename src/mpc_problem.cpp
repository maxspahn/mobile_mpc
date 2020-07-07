#include "mpc_problem.h"

MpcProblem::MpcProblem() : timeStep_(0.5)
{
  weights_ = weightArray({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
}

MpcProblem::~MpcProblem(){}

void MpcProblem::weights(weightArray w)
{
  weights_ = w;
}
weightArray MpcProblem::weights(){
  return weights_;
}

void MpcProblem::goal(goalArray g)
{
  goal_ = g;
}

goalArray MpcProblem::goal()
{
  return goal_;
}

void MpcProblem::param(int i, double val)
{
  params_[i] = val;
}

void MpcProblem::params(paramArray p)
{
  params_ = p;
}

paramArray MpcProblem::params()
{
  return params_;
}

void MpcProblem::curU(curUArray c)
{
  curU_ = c;
}

curUArray MpcProblem::curU()
{
  return curU_;
}

void MpcProblem::curState(curStateArray s)
{
  curState_ = s;
}

curStateArray MpcProblem::curState()
{
  return curState_;
}

void MpcProblem::setupParams()
{
  for (int i = 0; i < NX; ++i) {
    params_[i] = goal_[i];
  }
  for (int i = 0; i < NW; ++i) {
    params_[i+12] = weights_[i];
  }
}

void MpcProblem::setAcadoVariables(ACADOvariables& acadoVariables)
{
  setupParams();
  int i;
  for (i = 0; i < (NX + NS) * (N + 1); ++i)
  {
    // @todo Should be done using time horizon shift
    if (i < (NX + NS)) acadoVariables.x[i] = curState_[i];
    else acadoVariables.x[i] = 0.0;
  }
  for (i = 0; i < (NU * N); ++i)
  {
    if (i < NU) acadoVariables.u[i] = curU_[i];
    else acadoVariables.u[i] = 0.0;
  }
  for (i = 0; i < N; ++i)
  {
    for (int j = 0; j < NP; ++j)
    {
      acadoVariables.od[i * NP + j] = params_[j];
    }
  }
  for (i = 0; i < (NX + NS); ++i)
  {
    acadoVariables.x0[i] = curState_[i];
  }
}
  
