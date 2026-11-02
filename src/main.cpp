#include "simulation/Simulation.h"
#include "io/StateWriter.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

int main(int argc, char* argv[]) {
    const int particleCount = argc > 1 ? std::stoi(argv[1]) : 500;
    const unsigned seed = argc > 2 ? static_cast<unsigned>(std::stoul(argv[2])) : 42;
    const double side = argc > 3 ? std::stod(argv[3]) : 50.0;
    const std::string outputDirectory = argc > 4 ? argv[4] : "output";
    const int outputEverySteps = 1; // dt2 = 0.1 s for the current CPM parameters.

    std::filesystem::create_directories(outputDirectory);
    simulation::Simulation simulation(particleCount, seed, types::Board(side, 0.8));
    io::StateWriter writer(outputDirectory + "/states.txt");
    std::ofstream population(outputDirectory + "/population.txt");
    if (!population) {
        throw std::runtime_error("Could not open population output file.");
    }
    population.precision(std::numeric_limits<double>::max_digits10);
    population << "# time inside active\n";
    population << simulation.getTime() << ' ' << simulation.getPopulation() << ' '
               << simulation.getParticles().size() << '\n';
    writer.writeState(simulation.getParticles(), simulation.getTime());

    int steps = 0;
    while (!simulation.getParticles().empty()) {
        simulation.step();
        ++steps;
        population << simulation.getTime() << ' ' << simulation.getPopulation() << ' '
                   << simulation.getParticles().size() << '\n';
        if (steps % outputEverySteps == 0 || simulation.getParticles().empty()) {
            writer.writeState(simulation.getParticles(), simulation.getTime());
        }
    }
    population.flush();
    if (!population) {
        throw std::runtime_error("Could not write population output.");
    }

    std::cout << "Evacuation time: " << simulation.getEvacuationTime() << " s\n"
              << "Last removal time: " << simulation.getTime() << " s\n";
}
