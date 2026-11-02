#pragma once

#include "types/Board.h"
#include "types/Particle.h"
#include "neighbours/Grid.h"

#include <vector>

namespace simulation {

class Simulation {
public:
    Simulation(int particleCount = 500, unsigned seed = 42, types::Board board = {});

    void step();

    const types::Board& getBoard() const { return board; }
    const std::vector<types::Particle>& getParticles() const { return particles; }
    double getTime() const { return time; }
    double getDt() const { return dt; }
    int getPopulation() const;
    double getEvacuationTime() const { return evacuationTime; }

private:
    void initializeParticles(int particleCount, unsigned seed);
    void findContacts(std::vector<bool>& contacts, std::vector<types::Vector2D>& directions) const;

    types::Board board;
    neighbours::Grid grid;
    std::vector<types::Particle> particles;
    double time = 0.0;
    double evacuationTime = 0.0;
    static constexpr double maxRadius = 0.25;
    static constexpr double minRadius = 0.10;
    static constexpr double desiredSpeed = 1.5;
    static constexpr double escapeSpeed = desiredSpeed;
    static constexpr double tau = 0.5;
    static constexpr double beta = 0.9;
    static constexpr double dt = minRadius / (2 * escapeSpeed);
};

}
