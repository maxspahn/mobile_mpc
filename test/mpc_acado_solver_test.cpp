#include <gmock/gmock.h>

#include "mpc_acado_solver.h"

#define EPSILON 1e-5 

using ::testing::DoubleNear;

class MpcAcadoSolverTest : public ::testing::Test {
  protected:
    void SetUp() override {
      mpcProblem_ = MpcProblem();
      mpcProblem_.goal(0, 1.0);
      mpcProblem_.goal(5, 1.5);
      curStateArray csa = curStateArray({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 2.5, 0.0});
      mpcProblem_.curState(csa);
      mpcSolver_ = MpcAcadoSolver(); 
    }
  
  // void TearDown() override {}
  MpcProblem mpcProblem_;
  MpcAcadoSolver mpcSolver_;
};

TEST_F(MpcAcadoSolverTest, testIterationStep)
{
  mpcSolver_.solveMPC(mpcProblem_);
  curUArray u_opt = mpcSolver_.getOptimalControl();
  EXPECT_THAT(u_opt[7], DoubleNear(0.0, EPSILON));
  for (int i = 0; i < 10; ++i) {
    std::cout << u_opt[i] << std::endl;
  }
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
