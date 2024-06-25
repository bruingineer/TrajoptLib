// Copyright (c) TrajoptLib contributors

#pragma once

#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "trajopt/geometry/Pose2.hpp"
#include "trajopt/geometry/Translation2.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Linear velocity direction equality constraint.
 */
class TRAJOPT_DLLEXPORT LinearVelocityDirectionConstraint {
 public:
  /**
   * Constructs a LinearVelocityDirectionConstraint.
   *
   * @param angle The angle (radians).
   */
  explicit LinearVelocityDirectionConstraint(double angle) : m_angle{angle} {}

  /**
   * Applies this constraint to the given problem.
   *
   * @param problem The optimization problem.
   * @param pose The robot's pose.
   * @param linearVelocity The robot's linear velocity.
   * @param angularVelocity The robot's angular velocity.
   */
  void Apply([[maybe_unused]] sleipnir::OptimizationProblem& problem,
             [[maybe_unused]] const Pose2v& pose,
             [[maybe_unused]] const Translation2v& linearVelocity,
             [[maybe_unused]] const sleipnir::Variable& angularVelocity) {
    // <v_x, v_y> and <u_x, u_y> must be parallel
    //
    //   (v ⋅ u)/‖v‖ = 1
    //   v ⋅ u = ‖v‖
    //   (v ⋅ u)² = ‖v‖²
    auto dot = linearVelocity.Dot(Translation2d{m_angle.Cos(), m_angle.Sin()});
    problem.SubjectTo(dot * dot == linearVelocity.SquaredNorm());
  }

 private:
  trajopt::Rotation2d m_angle;
};

}  // namespace trajopt