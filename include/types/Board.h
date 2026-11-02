#pragma once

#include "types/Vector2D.h"

#include <array>
#include <algorithm>

namespace types {

struct Wall {
    Vector2D start;
    Vector2D end;

    // Board walls are horizontal or vertical, with start <= end in both axes.
    Vector2D closestPoint(Vector2D position) const {
        return {std::clamp(position.x, start.x, end.x),
                std::clamp(position.y, start.y, end.y)};
    }
};

class Board {
public:
    Board(double side = 50.0, double doorWidth = 0.8);

    double getSide() const { return side; }
    double getDoorWidth() const { return doorWidth; }
    Vector2D getDoorTarget() const { return {side / 2, 0}; }
    Vector2D getExitTarget() const { return {side / 2, -1.0}; }
    const std::array<Wall, 5>& getWalls() const { return walls; }

private:
    double side;
    double doorWidth;
    std::array<Wall, 5> walls;
};

}
