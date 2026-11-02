#include "io/StateWriter.h"

#include <limits>
#include <stdexcept>

namespace io {

StateWriter::StateWriter(const std::string& path) : out(path) {
    if (!out) {
        throw std::runtime_error("Could not open output file: " + path);
    }
    out.precision(std::numeric_limits<double>::max_digits10);
    out << "# Frame: time particle_count\n"
        << "# Particle: id x y vx vy radius scheme (0=time, 1=event)\n";
}

void StateWriter::writeState(const std::vector<types::Particle>& particles, double time) {
    out << time << ' ' << particles.size() << '\n';
    for (const types::Particle& p : particles) {
        const types::Vector2D position = p.getPosition();
        const types::Vector2D velocity = p.getVelocity();
        out << p.getId() << ' ' << position.x << ' ' << position.y << ' '
            << velocity.x << ' ' << velocity.y << ' ' << p.getRadius() << ' '
            << (p.getScheme() == types::MotionScheme::EventDriven ? 1 : 0) << '\n';
    }
    out << '\n';
    if (!out) {
        throw std::runtime_error("Could not write simulation state.");
    }
}

}
