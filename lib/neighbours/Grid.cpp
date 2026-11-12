#include "neighbours/Grid.h"

#include <algorithm>

namespace neighbours {

Grid::Grid(double side, double minCellSize) {
    cellsPerSide = static_cast<int>(side / minCellSize);
    cellsPerSide = std::max(1, cellsPerSide);
    cellSize = side / cellsPerSide;
    cells.resize(cellsPerSide * cellsPerSide);
}

int Grid::cellIndex(double coordinate) const {
    int index = static_cast<int>(coordinate / cellSize);
    // Agents outside the door stay in the boundary cells until they are removed.
    return std::clamp(index, 0, cellsPerSide - 1);
}

void Grid::rebuild(const std::vector<types::Particle>& particles) {
    for (std::size_t cell : occupied) {
        cells[cell].clear();
    }
    occupied.clear();
    for (std::size_t i = 0; i < particles.size(); ++i) {
        const types::Vector2D position = particles[i].getPosition();
        const std::size_t cell = static_cast<std::size_t>(cellIndex(position.y) * cellsPerSide + cellIndex(position.x));
        if (cells[cell].empty()) {
            occupied.push_back(cell);
        }
        cells[cell].push_back(i);
    }
}

std::vector<std::size_t> Grid::getCandidates(types::Vector2D position) const {
    std::vector<std::size_t> candidates;
    const int cx = cellIndex(position.x);
    const int cy = cellIndex(position.y);
    for (int y = std::max(0, cy - 1); y <= std::min(cellsPerSide - 1, cy + 1); ++y) {
        for (int x = std::max(0, cx - 1); x <= std::min(cellsPerSide - 1, cx + 1); ++x) {
            const std::vector<std::size_t>& cell = cells[y * cellsPerSide + x];
            candidates.insert(candidates.end(), cell.begin(), cell.end());
        }
    }
    return candidates;
}

}
