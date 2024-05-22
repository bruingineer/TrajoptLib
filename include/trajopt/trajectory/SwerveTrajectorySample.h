// Copyright (c) TrajoptLib contributors

#pragma once

#include <utility>
#include <vector>

#include "trajopt/trajectory/HolonomicTrajectorySample.h"
#include "trajopt/SymbolExports.h"

namespace trajopt {

/**
 * Swerve trajectory sample.
 */
class TRAJOPT_DLLEXPORT SwerveTrajectorySample : public HolonomicTrajectorySample {
 public:
  /// The x forces for each module.
  std::vector<double> moduleFX;

  /// The y forces for each module.
  std::vector<double> moduleFY;

  constexpr SwerveTrajectorySample() = default;

  /**
   * Construct a SwerveTrajectorySample.
   *
   * @param timestamp The timestamp.
   * @param x The x coordinate.
   * @param y The y coordinate.
   * @param heading The heading.
   * @param velocityX The velocity's x component.
   * @param velocityY The velocity's y component.
   * @param angularVelocity The angular velocity.
   */
  constexpr SwerveTrajectorySample(double timestamp, double x, double y,
                                   double heading, double velocityX,
                                   double velocityY, double angularVelocity,
                                   std::vector<double> moduleFX,
                                   std::vector<double> moduleFY)
      : HolonomicTrajectorySample(timestamp, x, y, heading, velocityX,
                                  velocityY, angularVelocity),
        moduleFX{std::move(moduleFX)},
        moduleFY{std::move(moduleFY)} {}
};

}  // namespace trajopt
