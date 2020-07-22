#include <gmock/gmock.h>

#include "forces_converter.h"
#include "mpc_problem.h"

#define EPSILON 1e-5 

using ::testing::DoubleNear;

class ForcesConverterTest : public ::testing::Test {
  protected:
    void SetUp() override {
      converter_ = ForcesConverter();
      mpcProblem_ = MpcProblem(0.2, 0.1); /* dt safetyMargin */
      mpcProblem_.goal(0, 4.2);
      mpcProblem_.goal(5, 1.5);
      curStateArray csa = curStateArray({1.2, -0.5, 0.433, 0, 0.2, -1.3, 0.1, -1.5, 2.5, 0.2});
      mpcProblem_.curState(csa);
    }
  
  // void TearDown() override {}
  ForcesConverter converter_;
  MpcProblem mpcProblem_;
};

TEST_F(ForcesConverterTest, testParamSetup)
{
  converter_.setupParams(mpcProblem_);
  forcesParamArray p = converter_.params();
  EXPECT_EQ(p.size(), NPF);
  EXPECT_THAT(p[0], DoubleNear(0.2, EPSILON));
  EXPECT_THAT(p[1], DoubleNear(0.08, EPSILON));
  EXPECT_THAT(p[2], DoubleNear(0.544, EPSILON));
}

TEST_F(ForcesConverterTest, paramsConfig)
{
  converter_.setupParams(mpcProblem_);
  converter_.setForcesVariables(mpcProblem_);
  mm_MPC_params mp = converter_.forces_params();
  EXPECT_THAT(mp.all_parameters[0], DoubleNear(0.2, EPSILON));
  EXPECT_THAT(mp.all_parameters[1], DoubleNear(0.08, EPSILON));
  EXPECT_THAT(mp.all_parameters[2], DoubleNear(0.544, EPSILON));
  EXPECT_THAT(mp.all_parameters[0 + NPF], DoubleNear(0.2, EPSILON));
}

TEST_F(ForcesConverterTest, paramsGoal)
{
  converter_.setupParams(mpcProblem_);
  converter_.setForcesVariables(mpcProblem_);
  mm_MPC_params mp = converter_.forces_params();
  EXPECT_THAT(mp.all_parameters[3], DoubleNear(4.2, EPSILON));
  EXPECT_THAT(mp.all_parameters[8], DoubleNear(1.5, EPSILON));
  // goal
  EXPECT_THAT(mp.all_parameters[5 * NPF + 3], DoubleNear(4.2, EPSILON));
}

TEST_F(ForcesConverterTest, paramsWeights)
{
  converter_.setupParams(mpcProblem_);
  converter_.setForcesVariables(mpcProblem_);
  mm_MPC_params mp = converter_.forces_params();

  //weights
  EXPECT_THAT(mp.all_parameters[13], DoubleNear(1.0, EPSILON));
  EXPECT_THAT(mp.all_parameters[14], DoubleNear(1.0, EPSILON));
  EXPECT_THAT(mp.all_parameters[15], DoubleNear(1.0, EPSILON));
  EXPECT_THAT(mp.all_parameters[16], DoubleNear(1.0, EPSILON));
  EXPECT_THAT(mp.all_parameters[17], DoubleNear(1.0, EPSILON));
  EXPECT_THAT(mp.all_parameters[18], DoubleNear(1.0, EPSILON));
}

TEST_F(ForcesConverterTest, paramsSafetyObstacles)
{
  converter_.setupParams(mpcProblem_);
  converter_.setForcesVariables(mpcProblem_);
  mm_MPC_params mp = converter_.forces_params();
  // safetyMargin
  EXPECT_THAT(mp.all_parameters[19 * NPF + 19], DoubleNear(0.1, EPSILON));
}

TEST_F(ForcesConverterTest, testVariablesSetXinit)
{
  converter_.setupParams(mpcProblem_);
  converter_.setForcesVariables(mpcProblem_);
  mm_MPC_params mp = converter_.forces_params();
  EXPECT_THAT(mp.xinit[0], DoubleNear(1.2, EPSILON));
  EXPECT_THAT(mp.xinit[1], DoubleNear(-0.5, EPSILON));
  EXPECT_THAT(mp.xinit[2], DoubleNear(0.433, EPSILON));
  EXPECT_THAT(mp.xinit[15], DoubleNear(0.0, EPSILON));
}

TEST_F(ForcesConverterTest, testVariablesSetX0)
{
  converter_.setupParams(mpcProblem_);
  converter_.setForcesVariables(mpcProblem_);
  mm_MPC_params mp = converter_.forces_params();
  EXPECT_THAT(mp.x0[0], DoubleNear(1.2, EPSILON));
  EXPECT_THAT(mp.x0[1], DoubleNear(-0.5, EPSILON));
  EXPECT_THAT(mp.x0[2], DoubleNear(0.433, EPSILON));
  EXPECT_THAT(mp.x0[2 + (NUF + NS + NX)], DoubleNear(0.433, EPSILON));
}
  

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
