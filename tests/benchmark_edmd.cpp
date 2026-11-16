// Verificación y rendimiento de la grilla del EDMD.
//
//   benchmark_edmd check                 grilla sin choques perdidos, para varios intervalos
//   benchmark_edmd stats                 grilla vs AllPairs, 10 semillas
//   benchmark_edmd interval              barrido del intervalo de reconstrucción (N=500)
//   benchmark_edmd scaling               escalamiento con N a densidad 0,2 /m2
//
// El tiempo de cómputo mide el motor desde la construcción hasta el último retiro,
// sin escribir archivos.

#include "simulation/Simulation.h"
#include "simulation/SimulationEDMD.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

struct RunResult {
    double evacuationTime = 0.0;
    double lastRemoval = 0.0;
    double seconds = 0.0;
    long events = 0;
    long stale = 0;
    long rebuilds = 0;
    long overlappingSamples = 0;    // muestras con algún par solapado (choque perdido)
    double worstOverlap = 0.0;
};

double now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Mayor solapamiento entre pares, en O(N^2). Solo para verificar.
double worstOverlapOf(const std::vector<types::Particle>& particles) {
    double worst = 0.0;
    for (std::size_t i = 0; i < particles.size(); ++i) {
        for (std::size_t j = i + 1; j < particles.size(); ++j) {
            const double distance = (particles[i].getPosition() - particles[j].getPosition()).norm();
            worst = std::max(worst, particles[i].getRadius() + particles[j].getRadius() - distance);
        }
    }
    return worst;
}

RunResult runEdmd(int n, unsigned seed, double side, simulation::NeighbourSearch search,
                  double interval, bool checkOverlaps) {
    const types::Board board(side, 0.8);
    RunResult result;
    const double start = now();
    simulation::SimulationEDMD engine(n, seed, board, search, interval);
    while (!engine.getParticles().empty()) {
        engine.step();
        if (checkOverlaps) {
            const double overlap = worstOverlapOf(engine.getParticles());
            result.worstOverlap = std::max(result.worstOverlap, overlap);
            if (overlap > 1e-6) {
                ++result.overlappingSamples;
            }
        }
    }
    result.seconds = now() - start;
    result.evacuationTime = engine.getEvacuationTime();
    result.lastRemoval = engine.getTime();
    result.events = engine.getProcessedEvents();
    result.stale = engine.getStaleEvents();
    result.rebuilds = engine.getRebuilds();
    return result;
}

RunResult runTsdmd(int n, unsigned seed, double side) {
    const types::Board board(side, 0.8);
    RunResult result;
    const double start = now();
    simulation::Simulation engine(n, seed, board);
    while (!engine.getParticles().empty()) {
        engine.step();
    }
    result.seconds = now() - start;
    result.evacuationTime = engine.getEvacuationTime();
    result.lastRemoval = engine.getTime();
    return result;
}

void meanAndDeviation(const std::vector<double>& values, double& mean, double& deviation) {
    mean = 0.0;
    for (double v : values) {
        mean += v / static_cast<double>(values.size());
    }
    double variance = 0.0;
    for (double v : values) {
        variance += (v - mean) * (v - mean) / static_cast<double>(values.size() - 1);
    }
    deviation = std::sqrt(variance);
}

// La grilla no debe perder choques: en EDMD nunca hay pares solapados.
int check() {
    int failures = 0;
    const double intervals[] = {0.05, 0.25, 1.0, 3.0};
    std::printf("Grilla: solapamientos en cada muestra (N=200, L=50, semilla 7)\n");
    for (double interval : intervals) {
        const RunResult r = runEdmd(200, 7, 50.0, simulation::NeighbourSearch::CellGrid, interval, true);
        const bool ok = r.overlappingSamples == 0;
        failures += ok ? 0 : 1;
        std::printf("  %s  intervalo %.2f s (alcance %.2f m): muestras con solapamiento=%ld, max=%.2e m, evacuacion=%.2f s\n",
                    ok ? "PASS" : "FAIL", interval, simulation::SimulationEDMD::neighbourReach(interval),
                    r.overlappingSamples, r.worstOverlap, r.evacuationTime);
    }
    std::printf("Grilla en recinto chico y denso (N=150, L=10, semilla 3)\n");
    const RunResult dense = runEdmd(150, 3, 10.0, simulation::NeighbourSearch::CellGrid, simulation::SimulationEDMD::defaultRebuildInterval, true);
    const bool okDense = dense.overlappingSamples == 0;
    failures += okDense ? 0 : 1;
    std::printf("  %s  muestras con solapamiento=%ld, max=%.2e m, evacuacion=%.2f s\n",
                okDense ? "PASS" : "FAIL", dense.overlappingSamples, dense.worstOverlap, dense.evacuationTime);
    return failures;
}

int stats() {
    const int realizations = 10;
    std::vector<double> grid;
    std::vector<double> all;
    std::printf("Grilla vs AllPairs: N=200, L=50, %d semillas\n", realizations);
    for (int r = 0; r < realizations; ++r) {
        const unsigned seed = 100u + static_cast<unsigned>(r);
        const RunResult g = runEdmd(200, seed, 50.0, simulation::NeighbourSearch::CellGrid, simulation::SimulationEDMD::defaultRebuildInterval, false);
        const RunResult a = runEdmd(200, seed, 50.0, simulation::NeighbourSearch::AllPairs, 0.25, false);
        grid.push_back(g.evacuationTime);
        all.push_back(a.evacuationTime);
        std::printf("  semilla %u: grilla %.2f s (%.2f s de computo)  AllPairs %.2f s (%.2f s de computo)\n",
                    seed, g.evacuationTime, g.seconds, a.evacuationTime, a.seconds);
    }
    double mg = 0.0;
    double sg = 0.0;
    double ma = 0.0;
    double sa = 0.0;
    meanAndDeviation(grid, mg, sg);
    meanAndDeviation(all, ma, sa);
    const double se = std::sqrt((sg * sg + sa * sa) / realizations);
    const bool ok = std::abs(mg - ma) < 3 * se;
    std::printf("  grilla   media %.2f s, desvio %.2f s\n", mg, sg);
    std::printf("  AllPairs media %.2f s, desvio %.2f s\n", ma, sa);
    std::printf("  %s  diferencia %.2f s (error estandar %.2f s)\n", ok ? "PASS" : "FAIL", mg - ma, se);
    return ok ? 0 : 1;
}

int interval() {
    std::printf("Barrido del intervalo de reconstruccion: N=500, L=50, semilla 100\n");
    const RunResult all = runEdmd(500, 100, 50.0, simulation::NeighbourSearch::AllPairs, 0.25, false);
    std::printf("  AllPairs:            %7.2f s de computo  eventos=%ld  obsoletos=%ld  evacuacion=%.2f s\n",
                all.seconds, all.events, all.stale, all.evacuationTime);
    const double intervals[] = {0.02, 0.05, 0.1, 0.25, 0.5, 1.0};
    for (double value : intervals) {
        const RunResult r = runEdmd(500, 100, 50.0, simulation::NeighbourSearch::CellGrid, value, false);
        std::printf("  grilla, Δ=%.2f s:    %7.2f s de computo  eventos=%ld  obsoletos=%ld  reconstrucciones=%ld  evacuacion=%.2f s\n",
                    value, r.seconds, r.events, r.stale, r.rebuilds, r.evacuationTime);
    }
    return 0;
}

int scaling() {
    std::printf("Escalamiento a densidad 0,2 /m2 (L = sqrt(N / 0,2)), semilla 100\n");
    std::printf("  %6s %7s %12s %12s %12s %10s %10s\n", "N", "L [m]", "TS evac [s]", "ED evac [s]",
                "TS comp [s]", "ED comp [s]", "ED/TS");
    const int sizes[] = {100, 200, 500, 1000, 2000};
    for (int n : sizes) {
        const double side = std::sqrt(n / 0.2);
        const RunResult ts = runTsdmd(n, 100, side);
        const RunResult ed = runEdmd(n, 100, side, simulation::NeighbourSearch::CellGrid,
                                     simulation::SimulationEDMD::defaultRebuildInterval, false);
        std::printf("  %6d %7.1f %12.2f %12.2f %12.2f %10.2f %10.1f\n", n, side, ts.evacuationTime,
                    ed.evacuationTime, ts.seconds, ed.seconds, ed.seconds / ts.seconds);
        std::fflush(stdout);
    }
    return 0;
}

}

int main(int argc, char* argv[]) {
    const std::string mode = argc > 1 ? argv[1] : "check";
    if (mode == "check") {
        return check();
    }
    if (mode == "stats") {
        return stats();
    }
    if (mode == "interval") {
        return interval();
    }
    if (mode == "scaling") {
        return scaling();
    }
    std::printf("Modos: check, stats, interval, scaling\n");
    return 1;
}
