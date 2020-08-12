#include "mpc_problem.h"

MpcProblem::MpcProblem(double timeStep, double safetyMargin) :
  timeStep_(timeStep),
  safetyMargin_(safetyMargin)
{
  weights_ = weightArray({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  configRobot_ = configArray({0.08, 0.544});
  initializeConstraints();
}

MpcProblem::MpcProblem() :
  timeStep_(0.5),
  safetyMargin_(0.1)
{
  weights_ = weightArray({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  configRobot_ = configArray({0.08, 0.544});
  initializeConstraints();
}

MpcProblem::~MpcProblem(){}

void MpcProblem::initializeConstraints()
{
  for (int i = 0; i < NO; ++i) {
    obstacles_[SO * i + 0] = 0;
    obstacles_[SO * i + 1] = 0;
    obstacles_[SO * i + 2] = 0;
    obstacles_[SO * i + 3] = -100;
  }
  for (int i = 0; i < NPLANES; ++i) {
    planes_[SPLANES * i + 0] = 0;
    planes_[SPLANES * i + 1] = 0;
    planes_[SPLANES * i + 2] = -50;
    planes_[SPLANES * i + 3] = 0;
    planes_[SPLANES * i + 4] = 5;
    planes_[SPLANES * i + 5] = -50;
    planes_[SPLANES * i + 6] = 5;
    planes_[SPLANES * i + 7] = 0;
    planes_[SPLANES * i + 8] = -50;
  }
  for (int i = 0; i < NINFPLA; ++i) {
    infPlanes_[SINFPLA * i + 0] = 0.0;
    infPlanes_[SINFPLA * i + 1] = 0.0;
    infPlanes_[SINFPLA * i + 2] = 1.0;
    infPlanes_[SINFPLA * i + 3] = -5.0;
  }
}

void MpcProblem::weights(weightArray w)
{
  weights_ = w;
}
weightArray MpcProblem::weights(){
  return weights_;
}

double MpcProblem::weight(int wI){
  return weights_[wI];
}

void MpcProblem::weight(int wI, double val){
  weights_[wI] = val;
}

void MpcProblem::goal(goalArray g)
{
  goal_ = g;
}

goalArray MpcProblem::goal()
{
  return goal_;
}

void MpcProblem::goal(int gI, double val)
{
  goal_[gI] = val;
}

double MpcProblem::goal(int gI)
{
  return goal_[gI];
}

void MpcProblem::param(int i, double val)
{
  params_[i] = val;
}

double MpcProblem::param(int i)
{
  return params_[i];
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

void MpcProblem::curU(int cI, double val)
{
  curU_[cI] = val;
}

double MpcProblem::curU(int cI)
{
  return curU_[cI];
}

void MpcProblem::curState(curStateArray s)
{
  curState_ = s;
}

curStateArray MpcProblem::curState()
{
  return curState_;
}

void MpcProblem::curState(int cI, double val)
{
  curState_[cI] = val;
}

double MpcProblem::curState(int cI)
{
  return curState_[cI];
}

void MpcProblem::obstacles(int limit, obstacleArray o)
{
  for (unsigned int i = 0; i < limit; ++i) {
    obstacles_[i] = o[i];
  }
}

void MpcProblem::obstacles(obstacleArray o)
{
  obstacles_ = o;
}

obstacleArray MpcProblem::obstacles()
{
  return obstacles_;
}

double MpcProblem::obstacle(int oI)
{
  return obstacles_[oI];
}

void MpcProblem::planes(planeArray p)
{
  planes_ = p;
}

planeArray MpcProblem::planes()
{
  return planes_;
}

double MpcProblem::plane(int pI)
{
  return planes_[pI];
}

void MpcProblem::infPlanes(infPlaneArray ip)
{
  infPlanes_ = ip;
}

infPlaneArray MpcProblem::infPlanes()
{
  return infPlanes_;
}

double MpcProblem::infPlane(int ipI)
{
  return infPlanes_[ipI];
}

void MpcProblem::infPlane(int ipI, double val)
{
  infPlanes_[ipI] = val;
}

void MpcProblem::configRobot(configArray c)
{
  configRobot_ = c;
}

configArray MpcProblem::configRobot()
{
  return configRobot_;
}

double MpcProblem::configRobot(int cI)
{
  return configRobot_[cI];
}

void MpcProblem::slackVar(double s)
{
  curState_[10] = s;
}

double MpcProblem::slackVar()
{
  return curState_[10];
}

void MpcProblem::slackVel(double sv)
{
  curU_[9] = sv;
}

double MpcProblem::slackVel()
{
  return curU_[9];
}

void MpcProblem::timeStep(double ts)
{
  timeStep_ = ts;
}

double MpcProblem::timeStep()
{
  return timeStep_;
}

void MpcProblem::safetyMargin(double sm)
{
  safetyMargin_ = sm;
}

double MpcProblem::safetyMargin()
{
  return safetyMargin_;
}
/*
void MpcProblem::setupParams()
{
  for (int i = 0; i < NX; ++i) {
    params_[i] = goal_[i];
  }
  for (int i = 0; i < NW; ++i) {
    params_[i+12] = weights_[i];
  }
  for (int i = 0; i < NO; ++i) {
    params_[4*i+18+0] = obstacles_[4*i+0];
    params_[4*i+18+1] = obstacles_[4*i+1];
    params_[4*i+18+2] = obstacles_[4*i+2];
    params_[4*i+18+3] = obstacles_[4*i+3];
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

void MpcProblem::setForcesVariables(mm_MPC_params& params)
{
  for (int i = 0; i < (NX + NS); ++i) {
    params.xinit[i] = curState_[i];
  }
  for (int i = 0; i < NU; ++i) {
    params.xinit[i+(NX + NS)] = curU_[i];
  }
  int time_horizon = 19;
  int forcesParams = 20;
  for (int i = 0; i < time_horizon; ++i)
  {
    for (int j = 0; j < (NX + NS); ++j)
    {
      params.x0[i * (NX + NS + NU) + j] = curState_[j];
    }
    for (int j = 0; j < (NX + NS); ++j)
    {
      params.x0[i * (NX + NS + NU) + j + (NX + NS)] = curState_[j];
    }
    printf("Iteration %d and current index %d\n", i, i * forcesParams);
    params.all_parameters[i * (forcesParams) + 0] = timeStep_;
    printf("params.all_parameters[0] : %1.5f\n", params.all_parameters[0]);
    params.all_parameters[i * (forcesParams) + 1] = params_[10];
    params.all_parameters[i * (forcesParams) + 2] = params_[11];
    for (int j = 0; j < 10; ++j)
    {
      params.all_parameters[i * (forcesParams) + 3 + j] = params_[j];
    }
    params.all_parameters[i * (forcesParams) + 13] = params_[14];
    params.all_parameters[i * (forcesParams) + 14] = params_[12];
    params.all_parameters[i * (forcesParams) + 15] = params_[13];
    params.all_parameters[i * (forcesParams) + 16] = params_[17];
    params.all_parameters[i * (forcesParams) + 17] = params_[15];
    params.all_parameters[i * (forcesParams) + 18] = params_[16];
    // safetyMargin
    params.all_parameters[i * (forcesParams) + 19] = 0.0;
  }
      
  printf("params at the end %1.5f", params.all_parameters[0]);
}
*/
  
