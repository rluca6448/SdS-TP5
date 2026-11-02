#pragma once

#include "types/Particle.h"

#include <vector>
#include <cstddef>

namespace neighbours {

class Grid {
public:
    Grid(double side, double minCellSize);

    void rebuild(const std::vector<types::Particle>& particles);
    // Returns candidates from the same cell and the eight adjacent cells.
    std::vector<std::size_t> getCandidates(types::Vector2D position) const;

private:
    int cellIndex(double coordinate) const;

    int cellsPerSide;
    double cellSize;
    std::vector<std::vector<std::size_t>> cells;
};

}
