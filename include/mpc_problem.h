#ifndef MPC_PROBLEM_H
#define MPC_PROBLEM_H

#include <array>
#include <vector>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#define NX          10  /* Number of differential state variables.  */
#define NS          1   /* Number of slack variables. */
#define NU          10  /* Number of control inputs. */
#define NO          1   /* Numbero of obstacles. */
#define NP          22  /* Number of online data values. */
#define N           ACADO_N  /* Number of intervals in the horizon. */
#define NW          6   /* Number of weights. */


typedef std::array<double, NW> weightArray;
typedef std::array<double, NX> goalArray;
typedef std::array<double, NU> curUArray;
typedef std::array<double, NX + NS> curStateArray;
typedef std::array<double, NP> paramArray;
typedef std::array<double, NO * 4> obstacleArray;

class MpcProblem
{
private:
  weightArray weights_;
  goalArray goal_;
  paramArray params_;
  curUArray curU_;
  curStateArray curState_;
  obstacleArray obstacles_;
  double timeStep_;

public:
  MpcProblem();
  ~MpcProblem();
  void weights(weightArray);
  weightArray weights();
  void goal(goalArray);
  goalArray goal();
  void param(int, double);
  void params(paramArray);
  paramArray params();
  void curU(curUArray);
  curUArray curU();
  void curState(curStateArray);
  curStateArray curState();
  void slackVar(double);
  double slackVar();
  void slackVel(double);
  double slackVel();
  void obstacles(obstacleArray);
  obstacleArray obstacles();
  void setupParams();
  void setAcadoVariables(ACADOvariables&);
};

#endif /* MPC_PROBLEM_H */  
