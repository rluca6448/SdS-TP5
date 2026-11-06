#include "simulation/Simulation.h"
#include "simulation/CpmRules.h"

#include <random>
#include <algorithm>
#include <cmath>

namespace simulation {

Simulation::Simulation(int particleCount, unsigned seed, types::Board board)
    : board(board), grid(board.getSide(), 2 * maxRadius) {
    initializeParticles(particleCount, seed);
}

void Simulation::initializeParticles(int particleCount, unsigned seed) {
    std::mt19937 random(seed);
    std::uniform_real_distribution<double> coordinate(maxRadius, board.getSide() - maxRadius);
    particles.reserve(static_cast<std::size_t>(particleCount));

    while (particles.size() < static_cast<std::size_t>(particleCount)) {
        const types::Vector2D position{coordinate(random), coordinate(random)};
        bool overlaps = false;
        for (const types::Particle& particle : particles) {
            if ((position - particle.getPosition()).norm() <= maxRadius + particle.getRadius()) {
                overlaps = true;
                break;
            }
        }
        if (overlaps) {
            continue;
        }

        types::Particle particle(static_cast<int>(particles.size()), position, maxRadius);
        particle.setTarget(board.getDoorTarget());
        const types::Vector2D direction = particle.getTarget() - position;
        particle.setVelocity(direction.normalized() * desiredSpeed);
        particles.push_back(particle);
    }
}

void Simulation::findContacts(std::vector<bool>& contacts,
                              std::vector<types::Vector2D>& directions) const {
    for (std::size_t i = 0; i < particles.size(); ++i) {
        const types::Particle& particle = particles[i];
        for (std::size_t j : grid.getCandidates(particle.getPosition())) {
            if (j <= i) {
                continue;
            }
            const types::Vector2D delta = particle.getPosition() - particles[j].getPosition();
            if (delta.norm() <= particle.getRadius() + particles[j].getRadius()) {
                contacts[i] = contacts[j] = true;
                const types::Vector2D direction = delta.normalized();
                directions[i] += direction;
                directions[j] += direction * -1;
            }
        }

        for (const types::Wall& wall : board.getWalls()) {
            const types::Vector2D delta = particle.getPosition() - wall.closestPoint(particle.getPosition());
            if (delta.norm() <= particle.getRadius()) {
                contacts[i] = true;
                directions[i] += delta.normalized();
            }
        }
    }
}

void Simulation::step() {
    grid.rebuild(particles);
    std::vector<bool> contacts(particles.size(), false);
    std::vector<types::Vector2D> directions(particles.size());
    findContacts(contacts, directions);

    for (std::size_t i = 0; i < particles.size(); ++i) {
        types::Particle& particle = particles[i];
        if (contacts[i]) {
            particle.setRadius(minRadius);
            particle.setVelocity(directions[i].normalized() * escapeSpeed);
        } else {
            const double radius = cpm::recoveredRadius(particle.getRadius());
            particle.setRadius(radius);
            const double speed = cpm::speedForRadius(radius);
            const types::Vector2D direction = particle.getTarget() - particle.getPosition();
            particle.setVelocity(direction.normalized() * speed);
        }
        particle.setScheme(types::MotionScheme::TimeDriven);
    }

    for (types::Particle& particle : particles) {
        const types::Vector2D previous = particle.getPosition();
        const types::Vector2D position = previous + particle.getVelocity() * dt;
        particle.setPosition(position);
        if (previous.y > 0 && position.y <= 0) {
            const double fraction = previous.y / (previous.y - position.y);
            const double crossingX = previous.x + fraction * (position.x - previous.x);
            if (std::abs(crossingX - board.getSide() / 2) <= board.getDoorWidth() / 2) {
                particle.setTarget(board.getExitTarget());
                evacuationTime = time + dt;
            }
        }
    }
    particles.erase(std::remove_if(particles.begin(), particles.end(), [this](const types::Particle& particle) {
        return particle.getPosition().y <= board.getExitTarget().y;
    }), particles.end());
    time += dt;
}

int Simulation::getPopulation() const {
    int population = 0;
    for (const types::Particle& particle : particles) {
        if (particle.getPosition().y > 0) {
            ++population;
        }
    }
    return population;
}

}
