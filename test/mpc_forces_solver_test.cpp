#include <gmock/gmock.h>

#include "mpc_forces_solver.h"

#define EPSILON 1e-5 

using ::testing::DoubleNear;

class MpcForcesSolverTest : public ::testing::Test {
  protected:
    void SetUp() override {
      mpcProblem_ = MpcProblem(0.5, 0.3);
      goalArray ga = goalArray({0, -8.0, 1.5708, 1.0, 1.0, -1.5, -1.5, 0.5, 1.0, 0.2});
      curStateArray csa = curStateArray({-4.0, -4.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.5, 1.5, 0.0});
      weightArray wa = weightArray({1.0, 10.0, 0.0, 100.0, 0, 0});
      obstacleArray oa = obstacleArray({-1.5, -6, 0, 1});
      mpcProblem_.goal(ga);
      mpcProblem_.curState(csa);
      mpcProblem_.weights(wa);
      mpcProblem_.obstacles(oa);
      mpcSolver_ = MpcForcesSolver(); 
      mpcSolver_.setupMPC(mpcProblem_);
    }
  
  // void TearDown() override {}
  MpcProblem mpcProblem_;
  MpcForcesSolver mpcSolver_;
};

TEST_F(MpcForcesSolverTest, testSettingsXinit)
{
  mm_MPC_params fp = mpcSolver_.forces_params();
  EXPECT_THAT(fp.xinit[0], DoubleNear(-4.0, EPSILON));
  EXPECT_THAT(fp.xinit[1], DoubleNear(-4.0, EPSILON));
  EXPECT_THAT(fp.xinit[6], DoubleNear(-1.0, EPSILON));
  EXPECT_THAT(fp.xinit[7], DoubleNear(0.5, EPSILON));
  EXPECT_THAT(fp.xinit[8], DoubleNear(1.5, EPSILON));
  // U checking
  EXPECT_THAT(fp.xinit[15], DoubleNear(0.0, EPSILON));
}

TEST_F(MpcForcesSolverTest, testSettingsX0)
{
  mm_MPC_params fp = mpcSolver_.forces_params();
  EXPECT_THAT(fp.x0[0], DoubleNear(-4.0, EPSILON));
  EXPECT_THAT(fp.x0[1], DoubleNear(-4.0, EPSILON));
  EXPECT_THAT(fp.x0[6], DoubleNear(-1.0, EPSILON));
  EXPECT_THAT(fp.x0[7], DoubleNear(0.5, EPSILON));
  EXPECT_THAT(fp.x0[8], DoubleNear(1.5, EPSILON));
  EXPECT_THAT(fp.x0[1 * (NS + NX + NUF) + 0], DoubleNear(-4.0, EPSILON));
  EXPECT_THAT(fp.x0[4 * (NS + NX + NUF) + 1], DoubleNear(-4.0, EPSILON));
  EXPECT_THAT(fp.x0[2 * (NS + NX + NUF) + 6], DoubleNear(-1.0, EPSILON));
  EXPECT_THAT(fp.x0[7 * (NS + NX + NUF) + 7], DoubleNear(0.5, EPSILON));
  EXPECT_THAT(fp.x0[9 * (NS + NX + NUF) + 8], DoubleNear(1.5, EPSILON));
}

TEST_F(MpcForcesSolverTest, testSettingsParams)
{
  mm_MPC_params fp = mpcSolver_.forces_params();
  EXPECT_THAT(fp.all_parameters[20], DoubleNear(-1.5, EPSILON));
  EXPECT_THAT(fp.all_parameters[NPF + 13 + 3], DoubleNear(100.0, EPSILON));
}

TEST_F(MpcForcesSolverTest, testIterationStepOutput)
{
  mpcSolver_.solveMPC();
  curUArray u_opt = mpcSolver_.getOptimalControl();
  EXPECT_THAT(u_opt[0], DoubleNear(4.0, EPSILON));
  EXPECT_THAT(u_opt[1], DoubleNear(-1.65433, EPSILON));
  EXPECT_THAT(u_opt[2], DoubleNear(1.74, EPSILON));
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
