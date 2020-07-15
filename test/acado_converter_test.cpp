#include <gmock/gmock.h>

#include "acado_converter.h"
#include "mpc_problem.h"

#define EPSILON 1e-5 

using ::testing::DoubleNear;

class AcadoConverterTest : public ::testing::Test {
  protected:
    void SetUp() override {
      converter_ = AcadoConverter();
      mpcProblem_ = MpcProblem();
      mpcProblem_.goal(0, 4.2);
      mpcProblem_.goal(5, 1.5);
      curStateArray csa = curStateArray({1.2, -0.5, 0.433, 0, 0.2, -1.3, 0.1, -1.5, 2.5, 0.2});
      mpcProblem_.curState(csa);
    }
  
  // void TearDown() override {}
  AcadoConverter converter_;
  MpcProblem mpcProblem_;
};

TEST_F(AcadoConverterTest, testParamLength)
{
  EXPECT_EQ(converter_.np(), 22);
}

TEST_F(AcadoConverterTest, testParamSetup)
{
  converter_.setupParams(mpcProblem_);
  paramArray p = converter_.params();
  EXPECT_EQ(p.size(), 22);
  EXPECT_THAT(p[10], DoubleNear(0.08, EPSILON));
  EXPECT_THAT(p[0], DoubleNear(4.2, EPSILON));
  EXPECT_THAT(p[5], DoubleNear(1.5, EPSILON));
}

TEST_F(AcadoConverterTest, testAcadoSetupX0)
{
  converter_.setupParams(mpcProblem_);
  converter_.setAcadoVariables(mpcProblem_);
  ACADOvariables av = converter_.acadoVariables();
  EXPECT_THAT(av.x0[0], DoubleNear(1.2, EPSILON));
  EXPECT_THAT(av.x0[7], DoubleNear(-1.5, EPSILON));
}

TEST_F(AcadoConverterTest, testAcadoSetupSlack)
{
  converter_.setupParams(mpcProblem_);
  converter_.setAcadoVariables(mpcProblem_);
  ACADOvariables av = converter_.acadoVariables();
  EXPECT_THAT(av.x0[10], DoubleNear(0.0, EPSILON));
  mpcProblem_.slackVar(1.2);
  converter_.setAcadoVariables(mpcProblem_);
  av = converter_.acadoVariables();
  EXPECT_THAT(av.x0[10], DoubleNear(1.2, EPSILON));
}

TEST_F(AcadoConverterTest, testAcadoSetupX)
{
  converter_.setupParams(mpcProblem_);
  converter_.setAcadoVariables(mpcProblem_);
  ACADOvariables av = converter_.acadoVariables();
  EXPECT_THAT(av.x[0], DoubleNear(1.2, EPSILON));
  EXPECT_THAT(av.x[7], DoubleNear(-1.5, EPSILON));
  EXPECT_THAT(av.x[NX+NS+7], DoubleNear(-1.5, EPSILON));
  EXPECT_THAT(av.od[5], DoubleNear(1.5, EPSILON));
  EXPECT_THAT(av.od[NP + 5], DoubleNear(1.5, EPSILON));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
