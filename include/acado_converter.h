#ifndef ACADO_CONVERTER_H
#define ACADO_CONVERTER_H

#include "mpc_problem.h"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#define NP 22 /* NX + NC + NW + NO * 4 */

typedef std::array<double, NP> paramArray;

class AcadoConverter
{
private:
  int weightsIndexOffset_;
  int goalIndexOffset_;
  int obstaclesIndexOffset_;
  int configRobotOffset_;
  paramArray params_;
  ACADOvariables acadoVariables_;
  ACADOworkspace acadoWorkspace_;
  unsigned int np_;

public:
  AcadoConverter();
  void setupParams(MpcProblem);
  void setAcadoVariables(MpcProblem);
  unsigned int np();
  paramArray params();
  ACADOvariables acadoVariables();
  ACADOworkspace acadoWorkspace();
};

#endif /* ACADO_CONVERTER_H */
