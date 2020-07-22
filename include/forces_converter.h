#ifndef FORCES_CONVERTER_H
#define FORCES_CONVERTER_H

#include "mpc_problem.h"
#include "mm_MPC.h"

typedef std::array<double, NPF> forcesParamArray;

class ForcesConverter
{
private:
  int weightsIndexOffset_;
  int goalIndexOffset_;
  int obstaclesIndexOffset_;
  int planesIndexOffset_;
  int configRobotOffset_;
  int safetyMarginIndexOffset_;
  int timeStepIndexOffset_;
  forcesParamArray params_;
  mm_MPC_params forces_params_;
  mm_MPC_output forces_output_;
  mm_MPC_info forces_info;

public:
  ForcesConverter();
  void setupParams(MpcProblem);
  void setForcesVariables(MpcProblem);
  forcesParamArray params();
  mm_MPC_params forces_params();
};

#endif /* FORCES_CONVERTER_H */
