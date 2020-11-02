#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <gtest/gtest.h>
#include <fstream>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/utils/robot_model_test_utils.h>

class LoadPlanningModelsPr2 : public testing::Test
{
protected:
  void SetUp() override
  {
    moveit::core::RobotModelBuilder robot("floating_robot", "base_link");
    robot.addChain("base_link->ee", "floating");
    robot.addGroupChain("base_link", "ee", "group1");
    ASSERT_TRUE(robot.isValid());
    robot_model_ = robot.build();
  }

  void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelPtr robot_model_;
  // moveit::core::RobotModelPtr robot_wrist_model_;
};

inline Eigen::Quaterniond xyz_intrinsix_to_quat(double rx, double ry, double rz)
{
  return Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
}

inline Eigen::Quaterniond rotation_vector_to_quat(double rx, double ry, double rz)
{
  Eigen::Vector3d v{ rx, ry, rz };
  Eigen::Matrix3d m{ Eigen::AngleAxisd(v.norm(), v.normalized()) };
  return Eigen::Quaterniond{ m };
}

TEST_F(LoadPlanningModelsPr2, OrientationConstraintsParameterization)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.update();
  moveit::core::Transforms tf(robot_model_->getModelFrame());

  kinematic_constraints::OrientationConstraint oc(robot_model_);

  moveit_msgs::OrientationConstraint ocm;

  // center the orientation constraints around the current orientation of the link
  geometry_msgs::Pose p = tf2::toMsg(robot_state.getGlobalLinkTransform("ee"));
  ocm.orientation = p.orientation;

  ocm.link_name = "ee";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.absolute_x_axis_tolerance = 0.5;
  ocm.absolute_y_axis_tolerance = 0.5;
  ocm.absolute_z_axis_tolerance = 0.5;
  // ocm.parameterization = moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES;
  ocm.parameterization = moveit_msgs::OrientationConstraint::ROTATION_VECTOR;
  // ocm.parameterization = 3;
  ocm.weight = 1.0;

  EXPECT_TRUE(oc.configure(ocm, tf));

  // Constraints should be satisfied because we created them around the current orientation
  EXPECT_TRUE(oc.decide(robot_state, true).satisfied);

  // change the orientation of the end-effector to some fixed values
  // and check the constraints again
  // Eigen::Quaterniond quat = xyz_intrinsix_to_quat(0.1, 0.2, 0.3);
  Eigen::Quaterniond quat = rotation_vector_to_quat(0.1, 0.2, 0.3);
  Eigen::VectorXd joint_values(7);
  joint_values << 0.0, 0.0, 0.0, quat.x(), quat.y(), quat.z(), quat.w();
  robot_state.setJointGroupPositions("group1", joint_values);
  robot_state.update();

  EXPECT_FALSE(oc.decide(robot_state, true).satisfied);

  // oc.decide()
}

// TEST_F(LoadPlanningModelsPr2, OrientationConstraintsParameterization)
// {
//   moveit::core::RobotState robot_state(robot_model_);
//   robot_state.setToDefaultValues();
//   robot_state.update();
//   moveit::core::Transforms tf(robot_model_->getModelFrame());

//   kinematic_constraints::OrientationConstraint oc(robot_model_);

//   moveit_msgs::OrientationConstraint ocm;

//   // center the orientation constraints around the current orientation of the link
//   geometry_msgs::Pose p = tf2::toMsg(robot_state.getGlobalLinkTransform("r_wrist_roll_link"));
//   ocm.orientation = p.orientation;

//   ocm.link_name = "r_wrist_roll_link";
//   ocm.header.frame_id = robot_model_->getModelFrame();
//   // ocm.orientation.x = 0.0;
//   // ocm.orientation.y = 0.0;
//   // ocm.orientation.z = 0.0;
//   // ocm.orientation.w = 1.0;
//   ocm.absolute_x_axis_tolerance = 0.5;
//   ocm.absolute_y_axis_tolerance = 0.5;
//   ocm.absolute_z_axis_tolerance = 0.5;
//   // ocm.parameterization = moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES;
//   ocm.parameterization = 3;
//   ocm.weight = 1.0;

//   EXPECT_TRUE(oc.configure(ocm, tf));

//   // Constraints should be satisfied based on how we created them
//   EXPECT_TRUE(oc.decide(robot_state).satisfied);

//   // move a joint and check the constraints again
//   std::map<std::string, double> jvals;
//   jvals["r_wrist_flex_joint"] = 1.0;
//   robot_state.setVariablePositions(jvals);
//   robot_state.update();

//   EXPECT_FALSE(oc.decide(robot_state, true).satisfied);

//   //oc.decide()
// }

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
