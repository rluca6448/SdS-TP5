#include "simulation/Simulation.h"
#include "simulation/SimulationEDMD.h"
#include "simulation/SimulationHybrid.h"
#include "io/StateWriter.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

// Cantidad de agentes que avanzaron en TS en el último paso.
int countTimeDriven(const std::vector<types::Particle>& particles) {
    int count = 0;
    for (const types::Particle& particle : particles) {
        if (particle.getScheme() == types::MotionScheme::TimeDriven) {
            ++count;
        }
    }
    return count;
}

// Corre cualquiera de los tres motores (misma interfaz) y escribe estados y población.
// outputEverySteps = 0 no escribe states.txt (corridas en lote). El tiempo de motor
// mide solo step(): excluye la construcción del estado inicial y la escritura.
template <typename Engine>
void run(Engine& simulation, const std::string& outputDirectory, int outputEverySteps) {
    std::unique_ptr<io::StateWriter> writer;
    if (outputEverySteps > 0) {
        writer = std::make_unique<io::StateWriter>(outputDirectory + "/states.txt");
    }
    std::ofstream population(outputDirectory + "/population.txt");
    if (!population) {
        throw std::runtime_error("Could not open population output file.");
    }
    population.precision(std::numeric_limits<double>::max_digits10);
    population << "# time inside active timeDriven\n";
    population << simulation.getTime() << ' ' << simulation.getPopulation() << ' '
               << simulation.getParticles().size() << ' ' << countTimeDriven(simulation.getParticles()) << '\n';
    if (writer) {
        writer->writeState(simulation.getParticles(), simulation.getTime());
    }

    std::chrono::steady_clock::duration engineTime{};
    int steps = 0;
    while (!simulation.getParticles().empty()) {
        const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        simulation.step();
        engineTime += std::chrono::steady_clock::now() - start;
        ++steps;
        population << simulation.getTime() << ' ' << simulation.getPopulation() << ' '
                   << simulation.getParticles().size() << ' ' << countTimeDriven(simulation.getParticles()) << '\n';
        if (writer && (steps % outputEverySteps == 0 || simulation.getParticles().empty())) {
            writer->writeState(simulation.getParticles(), simulation.getTime());
        }
    }
    population.flush();
    if (!population) {
        throw std::runtime_error("Could not write population output.");
    }

    std::cout.precision(std::numeric_limits<double>::max_digits10);
    std::cout << "Evacuation time: " << simulation.getEvacuationTime() << " s\n"
              << "Last removal time: " << simulation.getTime() << " s\n"
              << "Engine time: " << std::chrono::duration<double>(engineTime).count() << " s\n";
}

int main(int argc, char* argv[]) {
    const int particleCount = argc > 1 ? std::stoi(argv[1]) : 500;
    const unsigned seed = argc > 2 ? static_cast<unsigned>(std::stoul(argv[2])) : 42;
    const double side = argc > 3 ? std::stod(argv[3]) : 50.0;
    const std::string outputDirectory = argc > 4 ? argv[4] : "output";
    const std::string scheme = argc > 5 ? argv[5] : "tsdmd";
    // Cada cuántos pasos de dt = 1/30 s se escribe states.txt: TP5_STATES_EVERY
    // (por defecto 1; 3 equivale a dt2 = 0,1 s; 0 no escribe estados).
    const char* statesEvery = std::getenv("TP5_STATES_EVERY");
    const int outputEverySteps = statesEvery != nullptr ? std::stoi(statesEvery) : 1;

    std::filesystem::create_directories(outputDirectory);
    const types::Board board(side, 0.8);
    if (scheme == "tsdmd") {
        simulation::Simulation simulation(particleCount, seed, board);
        run(simulation, outputDirectory, outputEverySteps);
    } else if (scheme == "edmd") {
        const std::string neighbours = argc > 6 ? argv[6] : "grid";
        const double rebuildInterval = argc > 7 ? std::stod(argv[7])
                                                : simulation::SimulationEDMD::defaultRebuildInterval;
        simulation::NeighbourSearch search = simulation::NeighbourSearch::CellGrid;
        if (neighbours == "allpairs") {
            search = simulation::NeighbourSearch::AllPairs;
        } else if (neighbours != "grid") {
            throw std::invalid_argument("Unknown neighbour search (use grid or allpairs): " + neighbours);
        }
        simulation::SimulationEDMD simulation(particleCount, seed, board, search, rebuildInterval);
        run(simulation, outputDirectory, outputEverySteps);
        std::cout << "Processed events: " << simulation.getProcessedEvents() << '\n'
                  << "Stale events: " << simulation.getStaleEvents() << '\n'
                  << "Grid rebuilds: " << simulation.getRebuilds() << '\n';
    } else if (scheme == "hybrid") {
        const int anticipation = argc > 6 ? std::stoi(argv[6]) : simulation::SimulationHybrid::defaultAnticipation;
        simulation::SimulationHybrid simulation(particleCount, seed, board, anticipation);
        run(simulation, outputDirectory, outputEverySteps);
        std::cout << "Predicted entries to TS: " << simulation.getPredictedEntries() << '\n'
                  << "Boundary entries to TS: " << simulation.getBoundaryEntries() << '\n'
                  << "Pair predictions: " << simulation.getPairPredictions() << '\n';
    } else {
        throw std::invalid_argument("Unknown scheme (use tsdmd, edmd or hybrid): " + scheme);
    }
}
