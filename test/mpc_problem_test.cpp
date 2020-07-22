#include <gmock/gmock.h>

#include "mpc_problem.h"

#define EPSILON 1e-5 

using ::testing::DoubleNear;

class MpcProblemTest : public ::testing::Test {
  protected:
    void SetUp() override {
      mpcProblem_ = MpcProblem();
    }
  
  // void TearDown() override {}
  MpcProblem mpcProblem_;
};

TEST_F(MpcProblemTest, checkDefaultValues)
{
  EXPECT_THAT(mpcProblem_.weight(5), DoubleNear(1.0, EPSILON));
  EXPECT_THAT(mpcProblem_.goal(5), DoubleNear(0.0, EPSILON));
  EXPECT_THAT(mpcProblem_.param(3), DoubleNear(0.0, EPSILON));
  EXPECT_THAT(mpcProblem_.param(14), DoubleNear(0.0, EPSILON));
}

TEST_F(MpcProblemTest, checkSetWeights)
{
  mpcProblem_.weight(3, 5.2);
  EXPECT_THAT(mpcProblem_.weight(5), DoubleNear(1.0, EPSILON));
  EXPECT_THAT(mpcProblem_.weight(3), DoubleNear(5.2, EPSILON));
}
  

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
