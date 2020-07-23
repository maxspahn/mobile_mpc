#ifndef MPC_PROBLEM_H
#define MPC_PROBLEM_H

#include <array>
#include <vector>

//#include "mm_MPC.h"

#define NC          2   /* Number of config parameters. */
#define NX          10  /* Number of differential state variables.  */
#define NS          1   /* Number of slack variables. */
#define NW          6   /* Number of weights. */
#define NU          10  /* Number of control inputs. */
#define NUF         9   /* Number of control inputs for forces. */
#define NO          0   /* Number of obstacles. */
#define SO          4   /* Size of obstacle data. */
#define NPLANES     0   /* Number of planes. */
#define SPLANES     9   /* Size of plane data. */
#define NINFPLA     15  /* Number of planes. */
#define SINFPLA     4   /* Size of plane data. */
/* Number of online data values.
 * NC + NX + NW + NO * SO + NPLANES * SPLANES + NINFPLA * SINFPLA*/
#define NP          78
#define NPF         80  /* Number of forces parameters. NP + 2 */
#define N           21  /* Number of intervals in the horizon. */
#define TH          20  /* Time horizon. */


typedef std::array<double, NW> weightArray;
typedef std::array<double, NX> goalArray;
typedef std::array<double, NU> curUArray;
typedef std::array<double, NX + NS> curStateArray;
typedef std::array<double, NP> paramArray;
typedef std::array<double, NO * SO> obstacleArray;
typedef std::array<double, NPLANES * SPLANES> planeArray;
typedef std::array<double, NINFPLA * SINFPLA> infPlaneArray;
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
  planeArray planes_;
  infPlaneArray infPlanes_;
  configArray configRobot_;
  double timeStep_;
  double safetyMargin_;

public:
  MpcProblem();
  MpcProblem(double, double);
  ~MpcProblem();
  void initializeConstraints();
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
  void planes(planeArray);
  planeArray planes();
  double plane(int);
  void infPlanes(infPlaneArray);
  infPlaneArray infPlanes();
  double infPlane(int);
  void infPlane(int, double);
  void configRobot(configArray);
  configArray configRobot();
  double configRobot(int);
  //void setupParams();
  //void setAcadoVariables(ACADOvariables&);
  //void setForcesVariables(mm_MPC_params&);
};

#endif /* MPC_PROBLEM_H */  
