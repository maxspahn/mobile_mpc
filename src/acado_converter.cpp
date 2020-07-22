#include "acado_converter.h"

AcadoConverter::AcadoConverter() : np_(NP)
{
  weightsIndexOffset_ = 12;
  goalIndexOffset_ = 0;
  obstaclesIndexOffset_ = 18;
  configRobotOffset_ = 10;
}

void AcadoConverter::setupParams(MpcProblem mp)
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
}

void AcadoConverter::setAcadoVariables(MpcProblem mp)
{
  for (int i = 0; i < (NX + NS); ++i) {
    acadoVariables_.x0[i] = mp.curState(i);
  }
  for (int timeStep = 0; timeStep < N; ++timeStep) {
    for (int i = 0; i < (NX + NS); ++i) {
      acadoVariables_.x[timeStep * (NX + NS) + i] = mp.curState(i);
    }
    for (int p = 0; p < NP; ++p) {
      acadoVariables_.od[timeStep * NP + p] = params_[p];
    }
  }
}

paramArray AcadoConverter::params()
{
  return params_;
}

ACADOvariables AcadoConverter::acadoVariables()
{
  return acadoVariables_;
}

ACADOworkspace AcadoConverter::acadoWorkspace()
{
  return acadoWorkspace_;
}

unsigned int AcadoConverter::np()
{
  return np_;
}
