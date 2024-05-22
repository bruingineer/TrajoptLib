// Copyright (c) TrajoptLib contributors

#pragma once

#include <utility>
#include <vector>

#include "trajopt/SymbolExports.h"
#include "trajopt/solution/SwerveSolution.h"
#include "trajopt/trajectory/SwerveTrajectorySample.h"

namespace trajopt {

/**
 * Swerve trajectory.
 */
class TRAJOPT_DLLEXPORT SwerveTrajectory {
 public:
  /// Trajectory samples.
  std::vector<SwerveTrajectorySample> samples;

  SwerveTrajectory() = default;

  /**
   * Construct a SwerveTrajectory from samples.
   *
   * @param samples The samples.
   */
  explicit SwerveTrajectory(std::vector<SwerveTrajectorySample> samples)
      : samples{std::move(samples)} {}

  /**
   * Construct a SwerveTrajectory from a solution.
   *
   * @param solution The solution.
   */
  explicit SwerveTrajectory(const SwerveSolution& solution) {
    double ts = 0.0;
    for (size_t samp = 0; samp < solution.x.size(); ++samp) {
      if (samp != 0) {
        ts += solution.dt[samp - 1];
      }
      samples.emplace_back(ts, solution.x[samp], solution.y[samp],
                           solution.theta[samp], solution.vx[samp],
                           solution.vy[samp], solution.omega[samp],
                           solution.moduleFX[samp], solution.moduleFY[samp]);
    }
  }
};

}  // namespace trajopt
