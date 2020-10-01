#ifndef MPC_PROBLEM_H
#define MPC_PROBLEM_H

#include <array>
#include <vector>
#include <iostream>

#if STATIC_MPC == 0
  #include "definitions_mm_mpc.h"
#else
  #include "definitions_mm_mpc_static_spheres.h"
#endif

typedef std::array<double, NW> weightArray;
typedef std::array<double, NX> goalArray;
typedef std::array<double, NU> curUArray;
typedef std::array<double, NX + NS> curStateArray;
typedef std::array<double, NP> paramArray;
typedef std::array<double, NO * SO> obstacleArray;
typedef std::array<double, NMO * SMO> movingObstacleArray;
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
  movingObstacleArray movingObstacles_;
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
  void timeStep(double);
  double timeStep();
  void safetyMargin(double);
  double safetyMargin();
  void obstacles(obstacleArray);
  void obstacles(int, obstacleArray);
  obstacleArray obstacles();
  double obstacle(int);
  void movingObstacles(movingObstacleArray);
  void movingObstacles(int, movingObstacleArray);
  void movingObstacle(int, double);
  movingObstacleArray movingObstacles();
  double movingObstacle(int);
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
