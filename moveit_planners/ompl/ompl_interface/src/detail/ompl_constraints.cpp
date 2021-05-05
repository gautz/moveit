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

// void BaseConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
//                               Eigen::Ref<Eigen::MatrixXd> out) const
// {
//   Eigen::VectorXd constraint_error = calcError(joint_values);
//   Eigen::VectorXd constraint_derivative = bounds_.derivative(constraint_error);
//   Eigen::MatrixXd robot_jacobian = calcErrorJacobian(joint_values);
//   for (std::size_t i{ 0 }; i < bounds_.size(); ++i)
//   {
//     out.row(i) = constraint_derivative[i] * robot_jacobian.row(i);
//   }
// }

/******************************************
 * Position Box constraints
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
 * Position Equality constraints
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
 * Linear System constraints
 * ****************************************/
LinearSystemPositionConstraint::LinearSystemPositionConstraint(const robot_model::RobotModelConstPtr& robot_model,
                                                       const std::string& group, const unsigned int num_dofs)
  : BaseConstraint(robot_model, group, num_dofs)
{
}

void LinearSystemPositionConstraint::parseConstraintMsg(const moveit_msgs::Constraints& constraints)
{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Parsing linear system position constraint for OMPL constrained state space.");
  // bounds_.clear();

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

  ROS_INFO_STREAM_NAMED(LOGNAME, "Linear system constraint on x-position? " << (is_dim_constrained_[0] ? "yes" : "no"));
  ROS_INFO_STREAM_NAMED(LOGNAME, "Linear system constraint on y-position? " << (is_dim_constrained_[1] ? "yes" : "no"));
  ROS_INFO_STREAM_NAMED(LOGNAME, "Linear system constraint on z-position? " << (is_dim_constrained_[2] ? "yes" : "no"));

  knot_number_ =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.size()-1;

  // extract target position and orientation
  geometry_msgs::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
  target_position_ << position.x, position.y, position.z;
  tf2::fromMsg(constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).orientation,
                           target_orientation_);

  // extract target position and orientation
  geometry_msgs::Point position_start =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(1).position;
  start_position_ << position_start.x, position_start.y, position_start.z;
  geometry_msgs::Point position_end =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(2).position;
  end_position_ << position_end.x, position_end.y, position_end.z;

  link_name_ = constraints.position_constraints.at(0).link_name;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Position constraints applied to link: " << link_name_);
  ROS_INFO_STREAM_NAMED(LOGNAME, "Start position: " << start_position_);
  ROS_INFO_STREAM_NAMED(LOGNAME, "End position: " << end_position_);
  ROS_INFO_STREAM_NAMED(LOGNAME, "Target position: " << target_position_);
// ROS_INFO_STREAM_NAMED(LOGNAME, "Target orientation: " << target_orientation_);

  Eigen::Vector3d residual;
  Eigen::Vector3d cartesianPosition;
  if(knot_number_==2) // The position is contrained on a line passing through two points
  {
  cartesianPosition = start_position_;
  // (yb-ya)(z-za)-(zb-za)(y-ya)=0
  residual[0] = (end_position_.y() - start_position_.y())*(cartesianPosition.z() - start_position_.z())-(end_position_.z() - start_position_.z())*(cartesianPosition.y() - start_position_.y());
  // (zb-za)(x-xa)-(xb-xa)(z-za)=0
  residual[1] = (end_position_.z() - start_position_.z())*(cartesianPosition.x() - start_position_.x())-(end_position_.x() - start_position_.x())*(cartesianPosition.z() - start_position_.z());
  // (xb-xa)(y-ya)-(yb-ya)(x-xa)=0
  residual[2] = (end_position_.x() - start_position_.x())*(cartesianPosition.y() - start_position_.y())-(end_position_.y() - start_position_.y())*(cartesianPosition.x() - start_position_.x());
  // for (std::size_t dim{ 0 }; dim < 3; ++dim)
  // {
  //   if (is_dim_constrained_[dim]){}
  //     // out[dim] = residual[dim];  // equality constraint dimension
  //   else
  //     residual[dim] = 0.0;  // unbounded dimension
  // }
  ROS_INFO_STREAM_NAMED(LOGNAME, "Residual of the start position " << residual);
  cartesianPosition = end_position_;
  // (yb-ya)(z-za)-(zb-za)(y-ya)=0
  residual[0] = (end_position_.y() - start_position_.y())*(cartesianPosition.z() - start_position_.z())-(end_position_.z() - start_position_.z())*(cartesianPosition.y() - start_position_.y());
  // (zb-za)(x-xa)-(xb-xa)(z-za)=0
  residual[1] = (end_position_.z() - start_position_.z())*(cartesianPosition.x() - start_position_.x())-(end_position_.x() - start_position_.x())*(cartesianPosition.z() - start_position_.z());
  // (xb-xa)(y-ya)-(yb-ya)(x-xa)=0
  residual[2] = (end_position_.x() - start_position_.x())*(cartesianPosition.y() - start_position_.y())-(end_position_.y() - start_position_.y())*(cartesianPosition.x() - start_position_.x());
  // for (std::size_t dim{ 0 }; dim < 3; ++dim)
  // {
  //   if (is_dim_constrained_[dim]){}
  //     // out[dim] = residual[dim];  // equality constraint dimension
  //   else
  //     residual[dim] = 0.0;  // unbounded dimension
  // }
  ROS_INFO_STREAM_NAMED(LOGNAME, "Residual of the end position " << residual);
  }
  else if(knot_number_==3) // The position is contrained on a circle passing through 3 points
{
  geometry_msgs::Point position_interim =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(3).position;
  interim_position_ << position_interim.x, position_interim.y, position_interim.z;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Interim position: " << interim_position_);


  cartesianPosition = start_position_;
  Eigen::Vector3d center_position;

  // first vector u=(start_position_ - interim_position_)
  Eigen::Vector3d u;
  u = start_position_ - interim_position_;
  u.normalize();
  // second vector v=(end_position_ - interim_position_)
  Eigen::Vector3d v;
  v = end_position_ - interim_position_;
  v.normalize();
  // orthogonal vector n
  Eigen::Vector3d n = v.cross(u);
  n.normalize();
  cylinder_axis_ = n;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Cylinder Axis direction: " << cylinder_axis_);

  // plane passing through the 3 points
  Eigen::MatrixXd abcd(3,4);
  // plane orthogonal to cylinder_axis_ and passing through interim_position_
  abcd(0,0) = n[0];
  abcd(0,1) = n[1];
  abcd(0,2) = n[2];
  abcd(0,3) = -n.dot(interim_position_);
  // abcd(0,3) = -(n[0]*interim_position_.x()+n[1]*interim_position_.y()+n[2]*interim_position_.z());
  // Eigen::Vector3d a;
  // Eigen::Vector3d b;
  // Eigen::Vector3d c;
  // Eigen::Vector3d d;

  // a=u2v3-u3v2
  // abcd(0,0)=u[1]*v[2]-u[2]*v[1];
  // // abcd(0,0)=(start_position_.y() - interim_position_.y())*(end_position_.z() - interim_position_.z())-(start_position_.z() - interim_position_.z())*(end_position_.y() - interim_position_.y());
  // // b=u3v1-u1v3
  // abcd(0,1)=u[2]*v[0]-u[0]*v[2];
  // // abcd(0,1)=(start_position_.z() - interim_position_.z())*(end_position_.x() - interim_position_.x())-(start_position_.x() - interim_position_.x())*(end_position_.z() - interim_position_.z());
  // // c=u1v2-u2v1
  // abcd(0,2)=u[0]*v[1]-u[1]*v[0];
  // // abcd(0,2)=(start_position_.x() - interim_position_.x())*(end_position_.y() - interim_position_.y())-(start_position_.x() - interim_position_.x())*(end_position_.x() - interim_position_.x());
  // abcd(0,3)=- (abcd(0,0)*interim_position_.x()+abcd(0,1)*interim_position_.y()+abcd(0,2)*interim_position_.z());
  residual[0] = abcd(0,0)*cartesianPosition.x() + abcd(0,1)*cartesianPosition.y() + abcd(0,2)*cartesianPosition.z() + abcd(0,3);
  ROS_INFO_STREAM_NAMED(LOGNAME, "Residual of the start position " << residual);
  coeff_d_=abcd(0,3);

  Eigen::Vector3d middle_i_s;
  middle_i_s = 0.5*(start_position_ + interim_position_);
  Eigen::Vector3d middle_i_e;
  middle_i_e = 0.5*(end_position_ + interim_position_);
  // plane passing through middle_i_s and orthogonal to u
  // u[0]*(x-middle_i_s[0])+u[1]*(y+middle_i_s[1])+u[2]*(z+middle_i_s[2])=0
  abcd(1,0) = u[0];
  abcd(1,1) = u[1];
  abcd(1,2) = u[2];
  abcd(1,3) = -u.dot(middle_i_s);
  // abcd(1,3) = -(u[0]*middle_i_s[0]+u[1]*middle_i_s[1]+u[2]*middle_i_s[2]);
  // plane passing through middle_i_e and orthogonal to v
  // v[0]*(x-middle_i_e[0])+v[1]*(y+middle_i_e[1])+v[2]*(z+middle_i_e[2])
  abcd(2,0) = v[0];
  abcd(2,1) = v[1];
  abcd(2,2) = v[2];
  abcd(2,3) = -v.dot(middle_i_e);
  // abcd(2,3) = -(v[0]*middle_i_e[0]+v[1]*middle_i_e[1]+v[2]*middle_i_e[2]);
  ROS_INFO_STREAM_NAMED(LOGNAME, "Matrix of linear coeff: " << abcd);

  // Determinant
  // abcd << a,b,c,d;
  Eigen::Matrix3d abc;
  abc = abcd.block(0,0,3,3);
  double delta;
  delta = abc.determinant();
  ROS_INFO_STREAM_NAMED(LOGNAME, "Determinant abc: " << delta);
  Eigen::Matrix3d dbc;
  dbc << abcd.col(3),abcd.col(1),abcd.col(2);
  double delta_x;
  delta_x = dbc.determinant();
  Eigen::Matrix3d adc;
  adc << abcd.col(0),abcd.col(3),abcd.col(2);
  double delta_y;
  delta_y = adc.determinant();
  Eigen::Matrix3d abd;
  abd << abcd.col(0),abcd.col(1),abcd.col(3);
  double delta_z;
  delta_z = abd.determinant();

  // cylinder with axis passing through the circle center and same radius
  // center of the circle
  center_position[0]=-delta_x/delta;
  center_position[1]=-delta_y/delta;
  center_position[2]=-delta_z/delta;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Center position: " << center_position);
  center_position_=center_position;

  // Vector CI
  Eigen::Vector3d center_to_interimPosition;
  center_to_interimPosition = interim_position_-center_position;
  ROS_INFO_STREAM_NAMED(LOGNAME, "CI: " << center_to_interimPosition);
  // Radius: sqrt(())
  double radius;
  radius = center_to_interimPosition.norm();
  ROS_INFO_STREAM_NAMED(LOGNAME, "Radius: " << radius);
  // radius_=radius;
  // Distance from (C,n) to I
  Eigen::Vector3d ortho_n_CI;
  ortho_n_CI = n.cross(center_to_interimPosition);
  norm_ortho_n_CI_ = ortho_n_CI.norm();
  ROS_INFO_STREAM_NAMED(LOGNAME, "Distance from (C,n) to I: " << norm_ortho_n_CI_);


  // Vector CM
  Eigen::Vector3d center_to_cartesianPosition;
  center_to_cartesianPosition = cartesianPosition-center_position;
  ROS_INFO_STREAM_NAMED(LOGNAME, "CM: " << center_to_cartesianPosition);
  // Eigen::Vector3d ortho;
  // ortho = center_to_cartesianPosition.cross(n);
  // Eigen::Vector3d ortho1;
  // ortho1 = center_to_axis.cross(n);
  // ROS_INFO_STREAM_NAMED(LOGNAME, "Axis direction: " << ortho);
  // residual[1] = ortho.norm()*ortho.norm()-radius*radius*n.norm()*n.norm();
  // Cylinder of axis n, center C, and passing through I (and A and B)
  // ||CM x n||² = ||CI x n||² = ||CA x n||² = ||CB x n||²
  // Distance from (C,n) to M
  Eigen::Vector3d ortho_n_CM;
  ortho_n_CM = n.cross(center_to_cartesianPosition);
  double norm_n_CM;
  norm_n_CM = ortho_n_CM.norm();
  ROS_INFO_STREAM_NAMED(LOGNAME, "Distance from (C,n) to M: " << norm_n_CM);
  residual[1] = norm_n_CM-norm_ortho_n_CI_;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Residual of the start position " << residual);

  cartesianPosition = interim_position_;
  residual[0] = abcd(0,0)*cartesianPosition.x() + abcd(0,1)*cartesianPosition.y() + abcd(0,2)*cartesianPosition.z() + abcd(0,3);
    // Vector CM
  center_to_cartesianPosition = cartesianPosition-center_position;
  ROS_INFO_STREAM_NAMED(LOGNAME, "CM: " << center_to_cartesianPosition);
  // Cylinder of axis n, center C, and passing through I (and A and B)
  // ||CM x n||² = ||CI x n||² = ||CA x n||² = ||CB x n||²
  // Distance from (C,n) to M
  ortho_n_CM = n.cross(center_to_cartesianPosition);
  norm_n_CM = ortho_n_CM.norm();
  ROS_INFO_STREAM_NAMED(LOGNAME, "Distance from (C,n) to M: " << norm_n_CM);
  residual[1] = norm_n_CM-norm_ortho_n_CI_;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Residual of the interim position " << residual);

  cartesianPosition = end_position_;
  residual[0] = abcd(0,0)*cartesianPosition.x() + abcd(0,1)*cartesianPosition.y() + abcd(0,2)*cartesianPosition.z() + abcd(0,3);
    // Vector CM
  center_to_cartesianPosition = cartesianPosition-center_position;
  ROS_INFO_STREAM_NAMED(LOGNAME, "CM: " << center_to_cartesianPosition);
  // Cylinder of axis n, center C, and passing through I (and A and B)
  // ||CM x n||² = ||CI x n||² = ||CA x n||² = ||CB x n||²
  // Distance from (C,n) to M
  ortho_n_CM = n.cross(center_to_cartesianPosition);
  norm_n_CM = ortho_n_CM.norm();
  ROS_INFO_STREAM_NAMED(LOGNAME, "Distance from (C,n) to M: " << norm_n_CM);
  residual[1] = norm_n_CM-norm_ortho_n_CI_;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Residual of the end position " << residual);
  }
}

void LinearSystemPositionConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                          Eigen::Ref<Eigen::VectorXd> out) const
{
  Eigen::Vector3d cartesianPosition = target_orientation_.matrix().transpose() * forwardKinematics(joint_values).translation();
  // We don't need to rotate the target to fit in the constraints box, the cartesian constraints line is given in world coordinate frame
  // Eigen::Vector3d cartesianPosition = forwardKinematics(joint_values).translation();
  Eigen::Vector3d residual;
  Eigen::Vector3d center_position;
  if(knot_number_==2) // The position is contrained on a line passing through two points
  {
  // (yb-ya)(z-za)-(zb-za)(y-ya)=0
  out[0] = (end_position_.y() - start_position_.y())*(cartesianPosition.z() - start_position_.z())-(end_position_.z() - start_position_.z())*(cartesianPosition.y() - start_position_.y());
  // (zb-za)(x-xa)-(xb-xa)(z-za)=0
  out[1] = (end_position_.z() - start_position_.z())*(cartesianPosition.x() - start_position_.x())-(end_position_.x() - start_position_.x())*(cartesianPosition.z() - start_position_.z());
  // (xb-xa)(y-ya)-(yb-ya)(x-xa)=0
  // residual[2] = (end_position_.x() - start_position_.x())*(cartesianPosition.y() - start_position_.y())-(end_position_.y() - start_position_.y())*(cartesianPosition.x() - start_position_.x());
  out[2]=0;
  // for (std::size_t dim{ 0 }; dim < 3; ++dim)
  // {
  //   if (is_dim_constrained_[dim])
  //     out[dim] = residual[dim];  // equality constraint dimension
  //   else
  //     out[dim] = 0.0;  // unbounded dimension
  // }
  }
  else if(knot_number_==3) // The position is contrained on a circle passing through 3 points
  {
  // M belongs to plane orthogonal to cylinder_axis_
  out[0] = cylinder_axis_.x()*cartesianPosition.x() + cylinder_axis_.y()*cartesianPosition.y() + cylinder_axis_.z()*cartesianPosition.z() + coeff_d_;

  // Vector CM
  Eigen::Vector3d center_to_cartesianPosition;
  center_to_cartesianPosition = cartesianPosition-center_position_;
  // ROS_INFO_STREAM_NAMED(LOGNAME, "CM: " << center_to_cartesianPosition);
  // Cylinder of axis n, center C, and passing through I (and A and B)
  // ||CM x n||² = ||CI x n||² = ||CA x n||² = ||CB x n||²
  // Distance from (C,n) to M
  Eigen::Vector3d ortho_n_CM;
  ortho_n_CM = cylinder_axis_.cross(center_to_cartesianPosition);
  double norm_n_CM;
  norm_n_CM = ortho_n_CM.norm();
  // ROS_INFO_STREAM_NAMED(LOGNAME, "Distance from (C,n) to M: " << norm_n_CM);
  // M belongs to Cylinder of axis cylinder_axis_, center C, and passing through I (and A and B)
  out[1] = norm_n_CM-norm_ortho_n_CI_;
  // ROS_INFO_STREAM_NAMED(LOGNAME, "Residual of the start position " << residual);

  // out[0] = abcd_(0,0)*cartesianPosition.x() + abcd_(0,1)*cartesianPosition.y() + abcd_(0,2)*cartesianPosition.z() + abcd_(0,3);
  // // ROS_INFO_STREAM_NAMED(LOGNAME, "Residual of the start position " << residual);
  
  // // Vector CM
  // Eigen::Vector3d center_to_cartesianPosition;
  // center_to_cartesianPosition = cartesianPosition-center_position_;
  // // ROS_INFO_STREAM_NAMED(LOGNAME, "CM: " << center_to_cartesianPosition);
  // Eigen::Vector3d ortho;
  // ortho = cylinder_axis_.cross(center_to_cartesianPosition);
  // // ROS_INFO_STREAM_NAMED(LOGNAME, "Axis direction: " << ortho);
  // out[1] = ortho.norm()*ortho.norm()-radius_*radius_*cylinder_axis_.norm()*cylinder_axis_.norm();
  // // ROS_INFO_STREAM_NAMED(LOGNAME, "Residual of the start position " << residual);

  // center_position_ =center_position;
  out[2] = 0.0;  // unbounded dimension
  }
  else if(knot_number_>3) {
  ROS_INFO_STREAM_NAMED(LOGNAME, "Splines are currently not supported. Number of knots given: " << knot_number_);
  }
  else{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Linear system constraints require at least 2 knots. Number of knots given: " << knot_number_);
  }
}


// void LinearSystemPositionConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
//                                           Eigen::Ref<Eigen::MatrixXd> out) const
// {
//   out.setZero();
//   // Eigen::MatrixXd jac = target_orientation_.matrix().transpose() * robotGeometricJacobian(joint_values).topRows(3);
//   Eigen::MatrixXd jac = robotGeometricJacobian(joint_values).topRows(3);
//   Eigen::MatrixXd dresidual_dcartesianPosition;
//   // d residual[0] / d x = 0
//   dresidual_dcartesianPosition(0,0) = 0;
//   // d residual[1] / d x = (zb-za)
//   dresidual_dcartesianPosition(1,0) = end_position_.z() - start_position_.z();
//   // d residual[2] / d x = (ya-yb)
//   dresidual_dcartesianPosition(2,0) = start_position_.y() - end_position_.y();
//   // d residual[0] / d y = (za-zb)
//   dresidual_dcartesianPosition(0,1) = start_position_.z() - end_position_.z();
//   // d residual[1] / d y = 0
//   dresidual_dcartesianPosition(1,1) = 0;
//   // d residual[2] / d y = (xb-xa)
//   dresidual_dcartesianPosition(2,1) = end_position_.x() - start_position_.x();
//   // d residual[0] / d z = (yb-ya)
//   dresidual_dcartesianPosition(0,2) = end_position_.y() - start_position_.y();
//   // d residual[1] / d z = (xa-xb)
//   dresidual_dcartesianPosition(1,2) = start_position_.x() - end_position_.x();
//   // d residual[2] / d z = 0
//   dresidual_dcartesianPosition(2,2) = 0;
//   // Eigen::MatrixXd residuald = dresidual_dcartesianPosition * jac;
//   for (std::size_t dim{ 0 }; dim < 3; ++dim)
//   {
//     if (is_dim_constrained_[dim])
//       out.row(dim) = dresidual_dcartesianPosition.row(dim) * jac;  // equality constraint dimension
//   }
// }

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
      ROS_INFO_STREAM_NAMED(LOGNAME, "OMPL is using box pose constraints.");
      pose_con = std::make_shared<BoxPoseConstraint>(robot_model, group, num_dofs);
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
    else if (constraints.name == "linear_system_constraints")
    {
      ROS_INFO_STREAM_NAMED(LOGNAME, "OMPL is using position constraints from a linear system.");
      pos_con = std::make_shared<LinearSystemPositionConstraint>(robot_model, group, num_dofs);
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
