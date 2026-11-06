#pragma once

#include "simulation/CpmParameters.h"
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
    static constexpr double maxRadius = cpm::maxRadius;
    static constexpr double minRadius = cpm::minRadius;
    static constexpr double desiredSpeed = cpm::desiredSpeed;
    static constexpr double escapeSpeed = cpm::escapeSpeed;
    static constexpr double tau = cpm::tau;
    static constexpr double beta = cpm::beta;
    static constexpr double dt = cpm::dt;
};

}
