#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <gtest/gtest.h>
#include <fstream>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/utils/robot_model_test_utils.h>

class LoadPlanningModelsPr2 : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("pr2");
  }

  void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelPtr robot_model_;
};

TEST_F(LoadPlanningModelsPr2, OrientationConstraintsParameterization)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.update();
  moveit::core::Transforms tf(robot_model_->getModelFrame());

  kinematic_constraints::OrientationConstraint oc(robot_model_);

  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  // ocm.parameterization = moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES;
  ocm.parameterization = 3;
  ocm.weight = 1.0;

  EXPECT_TRUE(oc.configure(ocm, tf));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
