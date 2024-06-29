// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "frc/MathUtil.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/trajectory/TrajectoryParameterizer.h"
#include "spline/CubicHermitePoseSplineHolonomic.hpp"
#include "spline/SplineParameterizer.hpp"
#include "spline/SplineUtil.hpp"
#include "trajopt/geometry/Pose2.hpp"
#include "trajopt/path/Path.hpp"
#include "trajopt/util/TrajoptUtil.hpp"

namespace trajopt {

// TODO make control interval fn that is the first part of the below function
std::vector<frc::Trajectory> GenerateWaypointSplineTrajectories(
    const trajopt::SwervePath path,
    const std::vector<std::vector<Pose2d>> initialGuessPoints) {
  std::vector<trajopt::CubicHermitePoseSplineHolonomic> splines =
      CubicPoseControlVectorsFromWaypoints(initialGuessPoints);

  // Generate a parameterized spline
  std::vector<std::vector<frc::TrajectoryGenerator::PoseWithCurvature>>
      splinePoints;

  // Iterate through the vector and parameterize each spline
  for (auto&& spline : splines) {
    auto points = trajopt::SplineParameterizer::Parameterize(spline);
    splinePoints.push_back(points);
  }

  const auto maxWheelVelocity = units::meters_per_second_t(
      path.drivetrain.modules.front().wheelMaxAngularVelocity *
      path.drivetrain.modules.front().wheelRadius);

  wpi::array<frc::Translation2d, 4> moduleTranslations{wpi::empty_array};
  for (size_t i = 0; i < path.drivetrain.modules.size(); ++i) {
    const auto mod = path.drivetrain.modules.at(0);
    moduleTranslations.at(0) =
        frc::Translation2d{units::meter_t(mod.translation.X()),
                           units::meter_t(mod.translation.Y())};
  }
  const frc::SwerveDriveKinematics kinematics{
      moduleTranslations.at(0), moduleTranslations.at(1),
      moduleTranslations.at(2), moduleTranslations.at(3)};

  std::vector<frc::Trajectory> trajs;
  trajs.reserve(path.waypoints.size());
  size_t splineIdx = 0;
  for (size_t sgmtIdx = 1; sgmtIdx < initialGuessPoints.size(); ++sgmtIdx) {
    auto sgmtVel = maxWheelVelocity;
    const auto& sgmtGuessPoints = initialGuessPoints.at(sgmtIdx);
    for (size_t guessIdx = 0; guessIdx < sgmtGuessPoints.size(); ++guessIdx) {
      Pose2d start;
      Pose2d end = sgmtGuessPoints.at(guessIdx);
      if (guessIdx == 0) {
        start = initialGuessPoints.at(sgmtIdx - 1).back();
      } else {
        start = sgmtGuessPoints.at(guessIdx - 1);
      }
      auto dtheta =
          std::abs(frc::AngleModulus(
                       units::radian_t(std::abs(start.Rotation().Radians() -
                                                end.Rotation().Radians())))
                       .value());
      frc::Translation2d sgmtStart{units::meter_t(start.Translation().X()),
                                   units::meter_t(start.Translation().Y())};
      frc::Translation2d sgmtEnd{units::meter_t(end.Translation().X()),
                                 units::meter_t(end.Translation().Y())};

      for (auto& c : path.waypoints.at(sgmtIdx).segmentConstraints) {
        // assuming HolonomicVelocityConstraint with CircularSet2d
        if (std::holds_alternative<LinearVelocityMaxMagnitudeConstraint>(c)) {
          const auto& velocityHolonomicConstraint =
              std::get<LinearVelocityMaxMagnitudeConstraint>(c);
          auto vel = units::meters_per_second_t(
              velocityHolonomicConstraint.m_maxMagnitude);
          std::printf("max lin vel: %.2f - ", vel.value());
          if (vel < sgmtVel) {
            sgmtVel = vel;
          }
        } else if (std::holds_alternative<
                       AngularVelocityMaxMagnitudeConstraint>(c)) {
          const auto& angVelConstraint =
              std::get<AngularVelocityMaxMagnitudeConstraint>(c);
          auto maxAngVel = angVelConstraint.m_maxMagnitude;
          std::printf("max ang vel: %.2f - ", maxAngVel);
          // TODO add how the 1.5 is determined
          auto time = 1.5 * dtheta / maxAngVel;
          // estimating velocity for a straight line path
          // TODO use the spine path distance
          auto vel = sgmtStart.Distance(sgmtEnd) / units::second_t(time);
          if (vel < sgmtVel) {
            sgmtVel = vel;
          }
        }
      }
      std::printf("sgmtVel: %.2f\n", sgmtVel.value());
      frc::TrajectoryConfig sgmtConfig{sgmtVel, sgmtVel / units::second_t{1.0}};
      // uses each non-init guess waypoint as a stop point for first guess
      sgmtConfig.SetStartVelocity(0_mps);
      sgmtConfig.SetEndVelocity(0_mps);
      sgmtConfig.AddConstraint(
          frc::SwerveDriveKinematicsConstraint{kinematics, maxWheelVelocity});

      // specify parameterized spline points to use for trajectory
      const auto sgmtTraj = frc::TrajectoryParameterizer::
          TrajectoryParameterizer::TimeParameterizeTrajectory(
              splinePoints.at(++splineIdx - 1), sgmtConfig.Constraints(),
              sgmtConfig.StartVelocity(), sgmtConfig.EndVelocity(),
              sgmtConfig.MaxVelocity(), sgmtConfig.MaxAcceleration(),
              sgmtConfig.IsReversed());
      trajs.push_back(sgmtTraj);
    }
  }
  std::printf("path.wpt size: %zd\n", path.waypoints.size());
  std::printf("trajs size: %zd\n", trajs.size());
  return trajs;
}

std::vector<std::vector<frc::Trajectory::State>> CalculateWaypointStatesWithDt(
    const trajopt::SwervePath path,
    const std::vector<std::vector<Pose2d>> initialGuessPoints,
    const double desiredDt) {
  const auto trajs =
      GenerateWaypointSplineTrajectories(path, initialGuessPoints);

  size_t guessPoints = 0;
  for (const auto& guesses : initialGuessPoints) {
    guessPoints += guesses.size();
  }
  std::vector<std::vector<frc::Trajectory::State>> waypoint_states;
  waypoint_states.reserve(guessPoints);
  for (size_t i = 0; i < guessPoints; ++i) {
    waypoint_states.push_back(std::vector<frc::Trajectory::State>());
  }

  for (size_t sgmtIdx = 1; sgmtIdx < guessPoints; ++sgmtIdx) {
    // specify parameterized spline points to use for trajectory
    const auto sgmtTraj = trajs.at(sgmtIdx - 1);

    const auto wholeSgmtDt = sgmtTraj.TotalTime();
    const size_t samplesForSgmtNew = std::ceil(wholeSgmtDt.value() / desiredDt);
    const auto dt = wholeSgmtDt / samplesForSgmtNew;
    std::printf("dt for sgmt%zd with %zd samples: %.5f\n", sgmtIdx,
                samplesForSgmtNew, dt.value());

    if (sgmtIdx == 1) {
      std::printf("sgmt1\n");
      waypoint_states.at(sgmtIdx - 1).push_back(sgmtTraj.States().front());
    }

    for (size_t sampleIdx = 1; sampleIdx <= samplesForSgmtNew; ++sampleIdx) {
      auto t = static_cast<double>(sampleIdx) * dt;
      const auto point = sgmtTraj.Sample(t);
      waypoint_states.at(sgmtIdx).push_back(point);
      std::printf("%zd,", sampleIdx);
    }
    std::printf(" size: %zd\n", waypoint_states.at(sgmtIdx).size());
  }
  return waypoint_states;
}

std::vector<std::vector<frc::Trajectory::State>>
CalculateWaypointStatesWithControlIntervals(
    const trajopt::SwervePath path,
    const std::vector<std::vector<Pose2d>> initialGuessPoints,
    std::vector<size_t> controlIntervalCounts) {
  const auto trajs =
      GenerateWaypointSplineTrajectories(path, initialGuessPoints);

  size_t guessPoints = 0;
  for (const auto& guesses : initialGuessPoints) {
    guessPoints += guesses.size();
  }
  std::vector<std::vector<frc::Trajectory::State>> waypoint_states;
  waypoint_states.reserve(guessPoints);
  for (size_t i = 0; i < guessPoints; ++i) {
    waypoint_states.push_back(std::vector<frc::Trajectory::State>());
  }

  size_t trajIdx = 0;
  std::printf("sgmt1\n");
  waypoint_states.at(0).push_back(trajs.at(trajIdx).States().front());
  std::printf("ctrlCount: [");
  for (auto count : controlIntervalCounts) {
    std::printf("%zd,", count);
  }
  std::printf("]\n");
  for (size_t sgmtIdx = 1; sgmtIdx < initialGuessPoints.size(); ++sgmtIdx) {
    auto guessPointsSize = initialGuessPoints.at(sgmtIdx).size();
    auto samplesForSgmt = controlIntervalCounts.at(sgmtIdx - 1);
    size_t samples = samplesForSgmt / guessPointsSize;
    for (size_t guessIdx = 0; guessIdx < guessPointsSize; ++guessIdx) {
      if (guessIdx == (guessPointsSize - 1)) {
        samples += (samplesForSgmt % guessPointsSize);
      }
      for (size_t sampleIdx = 1; sampleIdx < samples + 1; ++sampleIdx) {
        auto t = trajs.at(trajIdx).TotalTime() *
                 static_cast<double>(sampleIdx) / samples;
        const auto state = trajs.at(trajIdx).Sample(t);
        waypoint_states.at(trajIdx + 1).push_back(state);
        // std::printf("%zd, x: %f, y: %f, t: %f\n",
        //               sampleIdx, state.pose.X().value(),
        //               state.pose.Y().value(), t.value());
      }
      std::printf(" size: %zd\n", waypoint_states.at(trajIdx + 1).size());
      ++trajIdx;
    }
  }
  return waypoint_states;
}

std::vector<size_t> CalculateControlIntervalCounts(
    const trajopt::SwervePath path,
    const std::vector<std::vector<Pose2d>> initialGuessPoints,
    const double desiredDt) {
  const auto desiredDt = 0.1;
  const auto trajectoriesSamples =
      CalculateWaypointStatesWithDt(path, initialGuessPoints, desiredDt);
  std::vector<size_t> counts;
  counts.reserve(path.waypoints.size());
  for (size_t i = 1; i < trajectoriesSamples.size(); ++i) {
    counts.push_back(trajectoriesSamples.at(i).size());
  }
  counts.push_back(1);
  return counts;
}

SwerveSolution CalculateSplineInitialGuessWithKinematicsAndConstraints(
    const trajopt::SwervePath path,
    const std::vector<std::vector<Pose2d>> initialGuessPoints,
    std::vector<size_t> controlIntervalCounts) {
  const auto trajectoriesSamples = CalculateWaypointStatesWithControlIntervals(
      path, initialGuessPoints, controlIntervalCounts);

  SwerveSolution initialGuess;
  for (const auto& traj : trajectoriesSamples) {
    auto dt = 0.1_s;
    if (traj.size() > 1) {
      dt = traj.at(1).t - traj.front().t;
    }
    for (const auto& point : traj) {
      // std::printf("point{%f, %f, %f, %f, %f}\n",
      //   point.pose.X().value(),
      //   point.pose.Y().value(),
      //   point.pose.Rotation().Cos(),
      //   point.pose.Rotation().Sin(),
      //   dt.value());
      initialGuess.x.push_back(point.pose.X().value());
      initialGuess.y.push_back(point.pose.Y().value());
      initialGuess.thetacos.push_back(point.pose.Rotation().Cos());
      initialGuess.thetasin.push_back(point.pose.Rotation().Sin());
      initialGuess.dt.push_back(dt.value());
    }
  }

  return initialGuess;
}

}  // namespace trajopt