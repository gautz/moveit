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
    //robot_model_ = moveit::core::loadTestingRobotModel("pr2");

    tf2::Quaternion q1, q2, q3;
    q1.setRPY(M_PI_2, -M_PI_2, 0.0);
    // q1 = tf2::Quaternion::getIdentity();

    q2.setRPY(0.0, M_PI_2, 0.0);
    // q2.setRPY(0.0, 0.0, M_PI_2);
    // q2 = tf2::Quaternion::getIdentity();

    // q3.setRPY(0.0, M_PI_2, 0.0);
    q3.setRPY(0.0, 0.0, -M_PI_2);
    // q3 = tf2::Quaternion::getIdentity();

    geometry_msgs::Pose p1, p2, p3;
    tf2::convert(q1, p1.orientation);
    tf2::convert(q2, p2.orientation);
    tf2::convert(q3, p3.orientation);

    moveit::core::RobotModelBuilder robot("robot_wrist", "base_link");
    robot.addChain("base_link->a->b->c", "revolute", {p1, p2, p3});
    // robot.addChain("c->tool", "fixed", {p1});
    robot.addGroupChain("base_link", "c", "wrist");
    ASSERT_TRUE(robot.isValid());
    robot_wrist_model_ = robot.build();
  }

  void TearDown() override
  {
  }

protected:
  //moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotModelPtr robot_wrist_model_;
};

TEST_F(LoadPlanningModelsPr2, OrientationConstraintsParameterization)
{
   moveit::core::RobotState robot_state(robot_wrist_model_);
  robot_state.setToDefaultValues();
  robot_state.update();
  moveit::core::Transforms tf(robot_wrist_model_->getModelFrame());

  kinematic_constraints::OrientationConstraint oc(robot_wrist_model_);

  moveit_msgs::OrientationConstraint ocm;

  // center the orientation constraints around the current orientation of the link
  geometry_msgs::Pose p = tf2::toMsg(robot_state.getGlobalLinkTransform("c"));
  ocm.orientation = p.orientation;

  ocm.link_name = "c";
  ocm.header.frame_id = robot_wrist_model_->getModelFrame();
  // ocm.orientation.x = 0.0;
  // ocm.orientation.y = 0.0;
  // ocm.orientation.z = 0.0;
  // ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.5;
  ocm.absolute_y_axis_tolerance = 0.5;
  ocm.absolute_z_axis_tolerance = 0.5;
  // ocm.parameterization = moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES;
  ocm.parameterization = 3;
  ocm.weight = 1.0;

  EXPECT_TRUE(oc.configure(ocm, tf));

  // Constraints should be satisfied based on how we created them
  EXPECT_TRUE(oc.decide(robot_state, true).satisfied);

  // move a joint and check the constraints again
  std::map<std::string, double> jvals;
  jvals["base_link-a-joint"] = 0.0;
  jvals["a-b-joint"] = 0.5;
  jvals["b-c-joint"] = 0.5;
  robot_state.setVariablePositions(jvals);
  robot_state.update();

  EXPECT_FALSE(oc.decide(robot_state, true).satisfied);

  //oc.decide()

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
