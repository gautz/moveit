/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, KU Leuven
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of KU Leuven nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Jeroen De Maeyer */

#include <moveit/ompl_interface/detail/ompl_constraints.h>

#include <moveit/ompl_interface/detail/threadsafe_state_storage.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/Constraints.h>
#include <tf2_eigen/tf2_eigen.h>

namespace ompl_interface
{
constexpr char LOGNAME[] = "ompl_constraints";

Bounds::Bounds(const std::vector<double>& lower, const std::vector<double>& upper)
  : lower_(lower), upper_(upper), size_(lower.size())
{
  // how to report this in release mode??
  assert(lower_.size() == upper_.size());
}

Eigen::VectorXd Bounds::penalty(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  assert((long)lower_.size() == x.size());
  Eigen::VectorXd penalty(x.size());

  for (Eigen::Index i{ 0 }; i < x.size(); ++i)
  {
    if (x[i] < lower_[i])
      penalty[i] = lower_[i] - x[i];
    else if (x[i] > upper_[i])
      penalty[i] = x[i] - upper_[i];
    else
      penalty[i] = 0.0;
  }
  return penalty;
}

Eigen::VectorXd Bounds::derivative(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  assert((long)lower_.size() == x.size());
  Eigen::VectorXd derivative(x.size());

  for (Eigen::Index i{ 0 }; i < x.size(); ++i)
  {
    if (x[i] < lower_[i])
      derivative[i] = -1.0;
    else if (x[i] > upper_[i])
      derivative[i] = 1.0;
    else
      derivative[i] = 0.0;
  }
  return derivative;
}

std::size_t Bounds::size() const
{
  return size_;
}

std::ostream& operator<<(std::ostream& os, const ompl_interface::Bounds& bounds)
{
  os << "Bounds:\n";
  for (std::size_t i{ 0 }; i < bounds.size(); ++i)
  {
    os << "( " << bounds.lower_[i] << ", " << bounds.upper_[i] << " )\n";
  }
  return os;
}

/****************************
 * Base class for constraints
 * **************************/
BaseConstraint::BaseConstraint(const robot_model::RobotModelConstPtr& robot_model, const std::string& group,
                               const unsigned int num_dofs, const unsigned int num_cons_)
  : ompl::base::Constraint(num_dofs, num_cons_)
  , state_storage_(robot_model)
  , joint_model_group_(robot_model->getJointModelGroup(group))

{
}

void BaseConstraint::init(const moveit_msgs::Constraints& constraints)
{
  parseConstraintMsg(constraints);
}

Eigen::Isometry3d BaseConstraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  moveit::core::RobotState* robot_state = state_storage_.getStateStorage();
  robot_state->setJointGroupPositions(joint_model_group_, joint_values);
  return robot_state->getGlobalLinkTransform(link_name_);
}

Eigen::MatrixXd BaseConstraint::robotGeometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  moveit::core::RobotState* robot_state = state_storage_.getStateStorage();
  robot_state->setJointGroupPositions(joint_model_group_, joint_values);
  Eigen::MatrixXd jacobian;
  // return value (success) not used, could return a garbage jacobian.
  robot_state->getJacobian(joint_model_group_, joint_model_group_->getLinkModel(link_name_),
                           Eigen::Vector3d(0.0, 0.0, 0.0), jacobian);
  return jacobian;
}

void BaseConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              Eigen::Ref<Eigen::VectorXd> out) const
{
  Eigen::VectorXd current_values = calcError(joint_values);
  out = bounds_.penalty(current_values);
}

void BaseConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              Eigen::Ref<Eigen::MatrixXd> out) const
{
  Eigen::VectorXd constraint_error = calcError(joint_values);
  Eigen::VectorXd constraint_derivative = bounds_.derivative(constraint_error);
  Eigen::MatrixXd robot_jacobian = calcErrorJacobian(joint_values);
  for (std::size_t i{ 0 }; i < bounds_.size(); ++i)
  {
    out.row(i) = constraint_derivative[i] * robot_jacobian.row(i);
  }
}

/******************************************
 * Position constraints
 * ****************************************/
BoxConstraint::BoxConstraint(const robot_model::RobotModelConstPtr& robot_model, const std::string& group,
                             const unsigned int num_dofs)
  : BaseConstraint(robot_model, group, num_dofs)
{
}

void BoxConstraint::parseConstraintMsg(const moveit_msgs::Constraints& constraints)
{
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Parsing box position constraint for OMPL constrained state space.");
  assert(bounds_.size() == 0);
  bounds_ = positionConstraintMsgToBoundVector(constraints.position_constraints.at(0));
  ROS_DEBUG_NAMED(LOGNAME, "Parsed Box constraints");
  ROS_DEBUG_STREAM_NAMED(LOGNAME, bounds_);

  // extract target position and orientation
  geometry_msgs::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
  target_position_ << position.x, position.y, position.z;
  tf2::fromMsg(constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).orientation,
               target_orientation_);

  link_name_ = constraints.position_constraints.at(0).link_name;
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Position constraints applied to link: " << link_name_);
}

Eigen::VectorXd BoxConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  return target_orientation_.matrix().transpose() * (forwardKinematics(x).translation() - target_position_);
}

Eigen::MatrixXd BoxConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  return target_orientation_.matrix().transpose() * robotGeometricJacobian(x).topRows(3);
}

/******************************************
 * Equality constraints
 * ****************************************/
EqualityPositionConstraint::EqualityPositionConstraint(const robot_model::RobotModelConstPtr& robot_model,
                                                       const std::string& group, const unsigned int num_dofs)
  : BaseConstraint(robot_model, group, num_dofs)
{
}

void EqualityPositionConstraint::parseConstraintMsg(const moveit_msgs::Constraints& constraints)
{
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Parsing equality position constraint for OMPL constrained state space.");

  std::vector<double> dims = constraints.position_constraints.at(0).constraint_region.primitives.at(0).dimensions;

  is_dim_constrained_ = { false, false, false };
  for (std::size_t i{ 0 }; i < dims.size(); ++i)
  {
    if (dims[i] < EQUALITY_CONSTRAINT_THRESHOLD_)
    {
      if (dims[i] < getTolerance())
      {
        ROS_ERROR_NAMED(
            LOGNAME,
            "Dimension %li of position constraint is smaller than the tolerance used to evaluate the constraints. "
            "This will make all states invalid and planning will fail :( Please use a value between %f and %f. ",
            i, getTolerance(), EQUALITY_CONSTRAINT_THRESHOLD_);
      }
      is_dim_constrained_.at(i) = true;
    }
  }

  // extract target position and orientation
  geometry_msgs::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
  target_position_ << position.x, position.y, position.z;
  tf2::fromMsg(constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).orientation,
               target_orientation_);

  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Equality constraint on x-position? " << (is_dim_constrained_[0] ? "yes" : "no"));
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Equality constraint on y-position? " << (is_dim_constrained_[1] ? "yes" : "no"));
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Equality constraint on z-position? " << (is_dim_constrained_[2] ? "yes" : "no"));

  link_name_ = constraints.position_constraints.at(0).link_name;
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Position constraints applied to link: " << link_name_);
}

void EqualityPositionConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                          Eigen::Ref<Eigen::VectorXd> out) const
{
  Eigen::Vector3d error =
      target_orientation_.matrix().transpose() * (forwardKinematics(joint_values).translation() - target_position_);
  for (std::size_t dim{ 0 }; dim < 3; ++dim)
  {
    if (is_dim_constrained_[dim])
      out[dim] = error[dim];  // equality constraint dimension
    else
      out[dim] = 0.0;  // unbounded dimension
  }
}

void EqualityPositionConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                          Eigen::Ref<Eigen::MatrixXd> out) const
{
  out.setZero();
  Eigen::MatrixXd jac = target_orientation_.matrix().transpose() * robotGeometricJacobian(joint_values).topRows(3);
  for (std::size_t dim{ 0 }; dim < 3; ++dim)
  {
    if (is_dim_constrained_[dim])
      out.row(dim) = jac.row(dim);  // equality constraint dimension
  }
}

/******************************************
 * Orientation constraints
 * ****************************************/
OrientationConstraint::OrientationConstraint(const robot_model::RobotModelConstPtr& robot_model, const std::string& group,
                        const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs)
  {
  }

void OrientationConstraint::parseConstraintMsg(const moveit_msgs::Constraints& constraints)
{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Parsing Orientation constraints for OMPL constrained state space.");
  assert(bounds_.size() == 0);
  bounds_ = orientationConstraintMsgToBoundVector(constraints.orientation_constraints.at(0));
  ROS_INFO_NAMED(LOGNAME, "Parsed Orientation constraints");
  ROS_INFO_STREAM_NAMED(LOGNAME,  bounds_);
  // ROS_DEBUG_STREAM_NAMED(LOGNAME, "Parsed rx / roll constraints" << bounds_[0]);
  // ROS_DEBUG_STREAM_NAMED(LOGNAME, "Parsed ry / pitch constraints" << bounds_[1]);
  // ROS_DEBUG_STREAM_NAMED(LOGNAME, "Parsed rz / yaw constraints" << bounds_[2]);

  // extract target position and orientation
  // geometry_msgs::Point position =
  //     constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
  // target_position_ << position.x, position.y, position.z;
  tf2::fromMsg(constraints.orientation_constraints.at(0).orientation,
               target_orientation_);
  ROS_INFO_NAMED(LOGNAME, "Quaternion desired");
  ROS_INFO_STREAM_NAMED(LOGNAME,  target_orientation_.x());
  ROS_INFO_STREAM_NAMED(LOGNAME,  target_orientation_.y());
  ROS_INFO_STREAM_NAMED(LOGNAME,  target_orientation_.z());
  ROS_INFO_STREAM_NAMED(LOGNAME,  target_orientation_.w());

  link_name_ = constraints.orientation_constraints.at(0).link_name;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Orientation constraints applied to link: " << link_name_);
}


Eigen::VectorXd OrientationConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  Eigen::Matrix3d orientation_difference = forwardKinematics(x).linear().transpose() * target_orientation_;
  Eigen::AngleAxisd aa(orientation_difference);
  return aa.axis() * aa.angle();
}

Eigen::MatrixXd OrientationConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  Eigen::Matrix3d orientation_difference = forwardKinematics(x).linear().transpose() * target_orientation_;
  Eigen::AngleAxisd aa{ orientation_difference };
  return -angularVelocityToAngleAxis(aa.angle(), aa.axis()) * robotGeometricJacobian(x).bottomRows(3);
}

// /******************************************
//  * Position equality and Orientation box constraints
//  * ****************************************/
// PoseConstraint::PoseConstraint(const robot_model::RobotModelConstPtr& robot_model, const std::string& group,
//                         const unsigned int num_dofs)
//     : BaseConstraint(robot_model, group, num_dofs)
//   {
//   }

// void PoseConstraint::parseConstraintMsg(const moveit_msgs::Constraints& constraints)
// {
//   ROS_DEBUG_STREAM_NAMED(LOGNAME, "Parsing equality position constraint for OMPL constrained state space.");

//   std::vector<double> dims = constraints.position_constraints.at(0).constraint_region.primitives.at(0).dimensions;

//   is_dim_constrained_ = { false, false, false };
//   for (std::size_t i{ 0 }; i < dims.size(); ++i)
//   {
//     if (dims[i] < EQUALITY_CONSTRAINT_THRESHOLD_)
//     {
//       if (dims[i] < getTolerance())
//       {
//         ROS_ERROR_NAMED(
//             LOGNAME,
//             "Dimension %li of position constraint is smaller than the tolerance used to evaluate the constraints. "
//             "This will make all states invalid and planning will fail :( Please use a value between %f and %f. ",
//             i, getTolerance(), EQUALITY_CONSTRAINT_THRESHOLD_);
//       }
//       is_dim_constrained_.at(i) = true;
//     }
//   }

//   // extract target position and orientation
//   geometry_msgs::Point position =
//       constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
//   target_position_ << position.x, position.y, position.z;
//   // tf2::fromMsg(constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).orientation,
//   //              target_orientation_);

//   ROS_DEBUG_STREAM_NAMED(LOGNAME, "Equality constraint on x-position? " << (is_dim_constrained_[0] ? "yes" : "no"));
//   ROS_DEBUG_STREAM_NAMED(LOGNAME, "Equality constraint on y-position? " << (is_dim_constrained_[1] ? "yes" : "no"));
//   ROS_DEBUG_STREAM_NAMED(LOGNAME, "Equality constraint on z-position? " << (is_dim_constrained_[2] ? "yes" : "no"));

  
//   ROS_INFO_STREAM_NAMED(LOGNAME, "Parsing Orientation constraints for OMPL constrained state space.");
//   assert(bounds_.size() == 0);
//   bounds_ = orientationConstraintMsgToBoundVector(constraints.orientation_constraints.at(0));
//   ROS_INFO_NAMED(LOGNAME, "Parsed Orientation constraints");
//   ROS_INFO_STREAM_NAMED(LOGNAME,  bounds_);
//   // ROS_DEBUG_STREAM_NAMED(LOGNAME, "Parsed rx / roll constraints" << bounds_[0]);
//   // ROS_DEBUG_STREAM_NAMED(LOGNAME, "Parsed ry / pitch constraints" << bounds_[1]);
//   // ROS_DEBUG_STREAM_NAMED(LOGNAME, "Parsed rz / yaw constraints" << bounds_[2]);

//   // extract target position and orientation
//   // geometry_msgs::Point position =
//   //     constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
//   // target_position_ << position.x, position.y, position.z;
//   tf2::fromMsg(constraints.orientation_constraints.at(0).orientation,
//                target_orientation_);
//   ROS_INFO_NAMED(LOGNAME, "Quaternion desired");
//   ROS_INFO_STREAM_NAMED(LOGNAME,  target_orientation_.x());
//   ROS_INFO_STREAM_NAMED(LOGNAME,  target_orientation_.y());
//   ROS_INFO_STREAM_NAMED(LOGNAME,  target_orientation_.z());
//   ROS_INFO_STREAM_NAMED(LOGNAME,  target_orientation_.w());

//   link_name_ = constraints.position_constraints.at(0).link_name;
//   ROS_INFO_STREAM_NAMED(LOGNAME, "Pose constraints applied to link: " << link_name_);
// }


// Eigen::VectorXd PoseConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
// {
//   // Eigen::Matrix<double, 5, 1> error;
//   // Eigen::Matrix<double, 6, 1> error;
//   Eigen::VectorXd error;
//   error = target_orientation_.matrix().transpose() * (forwardKinematics(x).translation() - target_position_);
//   // error.assign(3,target_orientation_.matrix().transpose() * (forwardKinematics(x).translation() - target_position_));
//   // Eigen::Vector3d error =
//   //     target_orientation_.matrix().transpose() * (forwardKinematics(joint_values).translation() - target_position_);
//   // Eigen::Ref<Eigen::VectorXd> out;
//   for (std::size_t dim{ 0 }; dim < 3; ++dim)
//   {
//     if (is_dim_constrained_[dim])
//       error[dim] = error[dim];  // equality constraint dimension
//     else
//       error[dim] = 0.0;  // unbounded dimension
//   }

//   Eigen::Matrix3d orientation_difference = forwardKinematics(x).linear().transpose() * target_orientation_;
//   Eigen::AngleAxisd aa(orientation_difference);
//   // error.bottomRows(3) = aa.axis() * aa.angle();
//   error[3] = (aa.axis()[0] * aa.angle());
//   error[4] = (aa.axis()[0] * aa.angle());
//   // error.push_back(aa.axis()[0] * aa.angle()[0]);
//   // error.push_back(aa.axis()[1] * aa.angle()[1]);
//   return error;
// }

// Eigen::MatrixXd PoseConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
// {
//   target_orientation_.matrix().transpose() * robotGeometricJacobian(x).topRows(3);
//   Eigen::Matrix3d orientation_difference = forwardKinematics(x).linear().transpose() * target_orientation_;
//   Eigen::AngleAxisd aa{ orientation_difference };
//   return -angularVelocityToAngleAxis(aa.angle(), aa.axis()) * robotGeometricJacobian(x).bottomRows(3);
// }

/******************************************
 * Position box and Orientation box constraints
 * ****************************************/
BoxPoseConstraint::BoxPoseConstraint(const robot_model::RobotModelConstPtr& robot_model, const std::string& group,
                        const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs)
  {
  }

void BoxPoseConstraint::parseConstraintMsg(const moveit_msgs::Constraints& constraints)
{
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Parsing box position constraint for OMPL constrained state space.");
  assert(bounds_.size() == 0);
  Bounds bounds_pos = positionConstraintMsgToBoundVector(constraints.position_constraints.at(0));
  ROS_DEBUG_NAMED(LOGNAME, "Parsed Box constraints");
  ROS_DEBUG_STREAM_NAMED(LOGNAME, bounds_);

  // extract target position and orientation
  geometry_msgs::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
  target_position_ << position.x, position.y, position.z;

  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Parsing Orientation constraints for OMPL constrained state space.");
  // assert(bounds_.size() == 0);
  Bounds bounds_ori = orientationConstraintMsgToBoundVector(constraints.orientation_constraints.at(0));
  ROS_DEBUG_NAMED(LOGNAME, "Parsed Orientation constraints");
  ROS_DEBUG_STREAM_NAMED(LOGNAME,  bounds_);

  // extract target position and orientation
  // geometry_msgs::Point position =
  //     constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
  // target_position_ << position.x, position.y, position.z;
  tf2::fromMsg(constraints.orientation_constraints.at(0).orientation,
               target_orientation_);
  ROS_INFO_NAMED(LOGNAME, "Quaternion desired");
  ROS_INFO_STREAM_NAMED(LOGNAME,  target_orientation_.x());
  ROS_INFO_STREAM_NAMED(LOGNAME,  target_orientation_.y());
  ROS_INFO_STREAM_NAMED(LOGNAME,  target_orientation_.z());
  ROS_INFO_STREAM_NAMED(LOGNAME,  target_orientation_.w());
  std::vector<double> lower;
  std::vector<double> lower_pos;
  std::vector<double> lower_ori;
  lower_pos = bounds_pos.getLower();
  lower_ori = bounds_ori.getLower();
  lower.insert(lower.begin(),lower_pos.begin(),lower_pos.end());
  std::vector<double> upper;
  std::vector<double> upper_pos;
  std::vector<double> upper_ori;
  upper_pos = bounds_pos.getUpper();
  upper_ori = bounds_ori.getUpper();
  upper.insert(upper.begin(),upper_pos.begin(),upper_pos.end());
  bounds_ = Bounds(lower,upper);

  link_name_ = constraints.position_constraints.at(0).link_name;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Pose constraints applied to link: " << link_name_);
}


Eigen::VectorXd BoxPoseConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  Eigen::Vector3d error_pos;
  error_pos = target_orientation_.matrix().transpose() * (forwardKinematics(x).translation() - target_position_);
  
  Eigen::Vector3d error_ori;
  Eigen::Matrix3d orientation_difference = forwardKinematics(x).linear().transpose() * target_orientation_;
  Eigen::AngleAxisd aa(orientation_difference);
  error_ori = aa.axis() * aa.angle();
  // error[3] = (aa.axis()[0] * aa.angle());
  // error[4] = (aa.axis()[0] * aa.angle());
  // error.push_back(aa.axis()[0] * aa.angle()[0]);
  // error.push_back(aa.axis()[1] * aa.angle()[1]);

  Eigen::VectorXd error = Eigen::VectorXd(6);
  // Eigen::VectorXd error = Eigen::VectorXd(5);
  // error << error_pos, error_ori[0], error_ori[1];
  return error_pos, error_ori;
}

Eigen::MatrixXd BoxPoseConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  Eigen::MatrixXd positionErrorJacobian = target_orientation_.matrix().transpose() * robotGeometricJacobian(x).topRows(3);

  Eigen::Matrix3d orientation_difference = forwardKinematics(x).linear().transpose() * target_orientation_;
  Eigen::AngleAxisd aa{ orientation_difference };
  Eigen::MatrixXd orientationErrorJacobian = -angularVelocityToAngleAxis(aa.angle(), aa.axis()) * robotGeometricJacobian(x).bottomRows(3);
  Eigen::MatrixXd poseErrorJacobian = Eigen::MatrixXd(positionErrorJacobian.rows()+orientationErrorJacobian.rows(),positionErrorJacobian.cols());

  poseErrorJacobian << positionErrorJacobian, orientationErrorJacobian;
  return poseErrorJacobian;
}

/************************************
 * MoveIt constraint message parsing
 * **********************************/
Bounds positionConstraintMsgToBoundVector(const moveit_msgs::PositionConstraint& pos_con)
{
  auto dims = pos_con.constraint_region.primitives.at(0).dimensions;

  // dimension of -1 signifies unconstrained parameter, so set to infinity
  for (auto& dim : dims)
  {
    if (dim == -1)
      dim = std::numeric_limits<double>::infinity();
  }

  return { { -dims[0] / 2, -dims[1] / 2, -dims[2] / 2 }, { dims[0] / 2, dims[1] / 2, dims[2] / 2 } };
}

Bounds orientationConstraintMsgToBoundVector(const moveit_msgs::OrientationConstraint& ori_con)
{
  std::vector<double> dims{ ori_con.absolute_x_axis_tolerance, ori_con.absolute_y_axis_tolerance,
                            ori_con.absolute_z_axis_tolerance };

  // dimension of -1 signifies unconstrained parameter, so set to infinity
  for (auto& dim : dims)
  {
    if (dim == -1)
      dim = std::numeric_limits<double>::infinity();
  }
  // return { { -dims[0], dims[0] }, { -dims[1], dims[1] }, { -dims[2], dims[2] } };
  return { { -dims[0], -dims[1], -dims[2] }, { dims[0] , dims[1] , dims[2] } };
}

/******************************************
 * OMPL Constraints Factory
 * ****************************************/
std::shared_ptr<BaseConstraint> createOMPLConstraint(const robot_model::RobotModelConstPtr& robot_model,
                                                     const std::string& group,
                                                     const moveit_msgs::Constraints& constraints)
{
  std::size_t num_dofs = robot_model->getJointModelGroup(group)->getVariableCount();
  std::size_t num_pos_con = constraints.position_constraints.size();
  std::size_t num_ori_con = constraints.orientation_constraints.size();

  // This factory method contains template code to support different constraints, but only position constraints are
  // currently supported. The other options return a nullptr for now and should not be used.

  if (num_pos_con > 1)
  {
    ROS_WARN_NAMED(LOGNAME, "Only a single position constraints supported. Using the first one.");
  }
  if (num_ori_con > 1)
  {
    ROS_WARN_NAMED(LOGNAME, "Only a single orientation constraints supported. Using the first one.");
  }

  if (num_pos_con > 0 && num_ori_con > 0)
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Constraint name: " << constraints.name);
    BaseConstraintPtr pose_con;
    // if (constraints.name == "use_equality_constraints")
    // {
    //   ROS_INFO_STREAM_NAMED(LOGNAME, "OMPL is using equality pose constraints.");
    //   pose_con = std::make_shared<PoseConstraint>(robot_model, group, num_dofs);
    // }
    // else
    // {
      ROS_INFO_STREAM_NAMED(LOGNAME, "OMPL is using box pose constraints.");
      pose_con = std::make_shared<BoxPoseConstraint>(robot_model, group, num_dofs);
    // }
    pose_con->init(constraints);
    return pose_con;
  }
  else if (num_pos_con > 0)
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Constraint name: " << constraints.name);
    BaseConstraintPtr pos_con;
    if (constraints.name == "use_equality_constraints")
    {
      ROS_INFO_STREAM_NAMED(LOGNAME, "OMPL is using equality position constraints.");
      pos_con = std::make_shared<EqualityPositionConstraint>(robot_model, group, num_dofs);
    }
    else
    {
      ROS_INFO_STREAM_NAMED(LOGNAME, "OMPL is using box position constraints.");
      pos_con = std::make_shared<BoxConstraint>(robot_model, group, num_dofs);
    }
    pos_con->init(constraints);
    return pos_con;
  }
  else if (num_ori_con > 0)
  {
    ROS_INFO_NAMED(LOGNAME, "OMPL is using orientation constraints.");
    BaseConstraintPtr ori_con;
    ori_con = std::make_shared<OrientationConstraint>(robot_model, group, num_dofs);
    ori_con->init(constraints);
    return ori_con;
  }
  else
  {
    ROS_ERROR_NAMED(LOGNAME, "No path constraints found in planning request.");
    return nullptr;
  }
}
}  // namespace ompl_interface
