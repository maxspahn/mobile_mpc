#ifndef MPC_PROBLEM_H
#define MPC_PROBLEM_H

#include <array>
#include <vector>

//#include "mm_MPC.h"

#define NX          10  /* Number of differential state variables.  */
#define NS          1   /* Number of slack variables. */
#define NU          10  /* Number of control inputs. */
#define NO          1   /* Number of obstacles. */
#define SO          4   /* Size of obstacle data. */
#define NP          22  /* Number of online data values. */
#define N           21  /* Number of intervals in the horizon. */
#define NW          6   /* Number of weights. */
#define NC          2   /* Number of config parameters. */
#define TH          20  /* Time horizon. */


typedef std::array<double, NW> weightArray;
typedef std::array<double, NX> goalArray;
typedef std::array<double, NU> curUArray;
typedef std::array<double, NX + NS> curStateArray;
typedef std::array<double, NP> paramArray;
typedef std::array<double, NO * SO> obstacleArray;
typedef std::array<double, NC> configArray;

class MpcProblem
{
private:
  weightArray weights_;
  goalArray goal_;
  paramArray params_;
  curUArray curU_;
  curStateArray curState_;
  obstacleArray obstacles_;
  configArray configRobot_;
  double timeStep_;
  double safetyMargin_;

public:
  MpcProblem();
  MpcProblem(double, double);
  ~MpcProblem();
  void weights(weightArray);
  weightArray weights();
  void weight(int, double);
  double weight(int);
  void goal(goalArray);
  goalArray goal();
  void goal(int, double);
  double goal(int);
  void param(int, double);
  double param(int);
  void params(paramArray);
  paramArray params();
  void curU(curUArray);
  curUArray curU();
  void curU(int, double);
  double curU(int);
  void curState(curStateArray);
  curStateArray curState();
  void curState(int, double);
  double curState(int);
  void slackVar(double);
  double slackVar();
  void slackVel(double);
  double slackVel();
  double timeStep();
  double safetyMargin();
  void obstacles(obstacleArray);
  obstacleArray obstacles();
  double obstacle(int);
  void configRobot(configArray);
  configArray configRobot();
  double configRobot(int);
  //void setupParams();
  //void setAcadoVariables(ACADOvariables&);
  //void setForcesVariables(mm_MPC_params&);
};

#endif /* MPC_PROBLEM_H */  
