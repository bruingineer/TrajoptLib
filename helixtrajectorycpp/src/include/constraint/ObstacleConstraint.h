#pragma once

#include <fmt/format.h>

#include "obstacle/Obstacle.h"

namespace helixtrajectory {

class ObstacleConstraint {
public:
    Obstacle obstacle;

    ObstacleConstraint(const Obstacle& obstacle);
};
}

template<>
struct fmt::formatter<helixtrajectory::ObstacleConstraint> {

    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx);

    template<typename FormatContext>
    auto format(const helixtrajectory::ObstacleConstraint& obstacleConstraint,
            FormatContext& ctx);
};