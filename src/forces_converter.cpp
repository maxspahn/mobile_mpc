#include "forces_converter.h"

ForcesConverter::ForcesConverter()
{
  timeStepIndexOffset_ = 0;
  configRobotOffset_ = 1;
  goalIndexOffset_ = 3;
  weightsIndexOffset_ = 13;
  obstaclesIndexOffset_ = 20;
  safetyMarginIndexOffset_ = 19;
}

void ForcesConverter::setupParams(MpcProblem mp)
{
  for (int wI = 0; wI < NW; ++wI)
  {
    params_[wI + weightsIndexOffset_] = mp.weight(wI);
  }
  for (int gI = 0; gI < NX; ++gI)
  {
    params_[gI + goalIndexOffset_] = mp.goal(gI);
  }
  for (int oI = 0; oI < (NO * SO); ++oI) {
    params_[oI + obstaclesIndexOffset_] = mp.obstacle(oI);
  }
  for (int cI = 0; cI < NC; ++cI)
  {
    params_[cI + configRobotOffset_] = mp.configRobot(cI);
  }
  params_[timeStepIndexOffset_] = mp.timeStep();
  params_[safetyMarginIndexOffset_] = mp.safetyMargin();
}

void ForcesConverter::setForcesVariables(MpcProblem mp)
{
  unsigned int index;
  for (int sv = 0; sv < (NX + NS); ++sv) {
    forces_params_.xinit[sv] = mp.curState(sv);
  }
  for (int uv = 0; uv < NUF; ++uv) {
    forces_params_.xinit[NX + NS + uv] = mp.curU(uv);
  }
  for (int timeStep = 0; timeStep < TH; ++timeStep) {
    for (int sv = 0; sv < (NX + NS); ++sv) {
      index = timeStep * (NX + NS + NUF) + sv;
      forces_params_.x0[index] = mp.curState(sv);
    }
    for (int uv = 0; uv < NUF; ++uv) {
      index = timeStep * (NX + NS + NUF) + (NX + NS) + uv;
      forces_params_.x0[index] = mp.curU(uv);
    }
    for (int p = 0; p < NPF; ++p) {
      index = (timeStep * NPF) + p;
      forces_params_.all_parameters[index] = params_[p];
    }
  }
}

forcesParamArray ForcesConverter::params()
{
  return params_;
}

mm_MPC_params ForcesConverter::forces_params()
{
  return forces_params_;
}
