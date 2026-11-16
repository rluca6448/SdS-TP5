// Diagnóstico: cuánto se solapan los agentes en cada esquema, y cómo es el
// flujo por la puerta. Mide sobre los estados muestreados cada dt.
#include "simulation/Simulation.h"
#include "simulation/SimulationEDMD.h"

#include <cstdio>
#include <vector>

struct OverlapStats {
    long samples = 0;
    long overlappingPairs = 0;
    double depthSum = 0.0;
    double maxDepth = 0.0;
    long nearDoorPairs = 0;           // pares solapados a menos de 2 m de la puerta
    double nearDoorDepthSum = 0.0;
};

static void accumulate(const std::vector<types::Particle>& particles, double side, OverlapStats& stats) {
    ++stats.samples;
    for (std::size_t i = 0; i < particles.size(); ++i) {
        for (std::size_t j = i + 1; j < particles.size(); ++j) {
            const types::Vector2D d = particles[i].getPosition() - particles[j].getPosition();
            const double depth = particles[i].getRadius() + particles[j].getRadius() - d.norm();
            if (depth > 1e-9) {
                ++stats.overlappingPairs;
                stats.depthSum += depth;
                stats.maxDepth = std::max(stats.maxDepth, depth);
                const types::Vector2D door{side / 2, 0.0};
                if ((particles[i].getPosition() - door).norm() < 2.0) {
                    ++stats.nearDoorPairs;
                    stats.nearDoorDepthSum += depth;
                }
            }
        }
    }
}

template <typename Engine>
static void measure(const char* name, Engine& engine, double side) {
    OverlapStats stats;
    std::vector<double> crossings;
    int previousPopulation = engine.getPopulation();
    double radiusNearDoorSum = 0.0;
    long radiusNearDoorCount = 0;
    while (!engine.getParticles().empty()) {
        engine.step();
        accumulate(engine.getParticles(), side, stats);
        for (const types::Particle& p : engine.getParticles()) {
            const types::Vector2D door{side / 2, 0.0};
            if (p.getPosition().y > 0 && (p.getPosition() - door).norm() < 1.5) {
                radiusNearDoorSum += p.getRadius();
                ++radiusNearDoorCount;
            }
        }
        const int population = engine.getPopulation();
        for (int k = 0; k < previousPopulation - population; ++k) {
            crossings.push_back(engine.getTime());
        }
        previousPopulation = population;
    }
    // Caudal en régimen: entre el 20 % y el 80 % de los cruces.
    const std::size_t a = crossings.size() / 5;
    const std::size_t b = 4 * crossings.size() / 5;
    const double flow = static_cast<double>(b - a) / (crossings[b] - crossings[a]);
    std::printf("  %s\n", name);
    std::printf("    pares solapados por muestra: %.2f   profundidad media: %.4f m   max: %.4f m\n",
                static_cast<double>(stats.overlappingPairs) / stats.samples,
                stats.overlappingPairs > 0 ? stats.depthSum / stats.overlappingPairs : 0.0,
                stats.maxDepth);
    std::printf("    cerca de la puerta: pares solapados/muestra %.2f, profundidad media %.4f m\n",
                static_cast<double>(stats.nearDoorPairs) / stats.samples,
                stats.nearDoorPairs > 0 ? stats.nearDoorDepthSum / stats.nearDoorPairs : 0.0);
    std::printf("    radio medio a menos de 1,5 m de la puerta: %.4f m\n",
                radiusNearDoorSum / radiusNearDoorCount);
    std::printf("    caudal en regimen: %.3f agentes/s   evacuacion: %.2f s\n",
                flow, engine.getEvacuationTime());
}

int main() {
    const double side = 50.0;
    const types::Board board(side, 0.8);
    std::printf("N=200, L=50, semilla 100\n");
    simulation::Simulation ts(200, 100, board);
    measure("TSDMD", ts, side);
    simulation::SimulationEDMD ed(200, 100, board);
    measure("EDMD", ed, side);
}
