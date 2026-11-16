// Verificación del híbrido ED + TS contra el TSDMD.
//
//   verify_hybrid check       todo-TS idéntico al TSDMD; sin contactos perdidos; divergencia
//   verify_hybrid stats       evacuación híbrido vs TSDMD en 10 semillas
//   verify_hybrid cost        costo y fracción TS según k (N=500, L=50)
//   verify_hybrid scaling     escalamiento con N a densidad 0,2 /m2

#include "simulation/Simulation.h"
#include "simulation/SimulationEDMD.h"
#include "simulation/SimulationHybrid.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

int failures = 0;

void report(bool ok, const std::string& name) {
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", name.c_str());
    if (!ok) {
        ++failures;
    }
}

double now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Mayor diferencia entre dos listas de partículas (mismos ids en el mismo orden).
// Devuelve infinito si no coinciden los ids.
double maxDifference(const std::vector<types::Particle>& a, const std::vector<types::Particle>& b,
                     bool includeVelocity) {
    if (a.size() != b.size()) {
        return INFINITY;
    }
    double worst = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].getId() != b[i].getId()) {
            return INFINITY;
        }
        worst = std::max(worst, (a[i].getPosition() - b[i].getPosition()).norm());
        worst = std::max(worst, std::abs(a[i].getRadius() - b[i].getRadius()));
        if (includeVelocity) {
            worst = std::max(worst, (a[i].getVelocity() - b[i].getVelocity()).norm());
        }
    }
    return worst;
}

void checkAllTimeDriven(int n, double side, unsigned seed) {
    const types::Board board(side, 0.8);
    simulation::Simulation ts(n, seed, board);
    simulation::SimulationHybrid hybrid(n, seed, board, 1, true);
    double worst = maxDifference(ts.getParticles(), hybrid.getParticles(), true);
    long steps = 0;
    while (!ts.getParticles().empty() && worst == 0.0) {
        ts.step();
        hybrid.step();
        ++steps;
        worst = std::max(worst, maxDifference(ts.getParticles(), hybrid.getParticles(), true));
        if (ts.getTime() != hybrid.getTime() || ts.getEvacuationTime() != hybrid.getEvacuationTime()) {
            worst = INFINITY;
        }
    }
    const bool ok = worst == 0.0 && hybrid.getParticles().empty();
    report(ok, "todos en TS: identico al TSDMD en cada uno de los " + std::to_string(steps) +
                   " pasos (N=" + std::to_string(n) + ", L=" + std::to_string(static_cast<int>(side)) + ")");
}

void checkNoMissedContacts(int n, double side, unsigned seed, int k) {
    const types::Board board(side, 0.8);
    simulation::SimulationHybrid hybrid(n, seed, board, k);
    hybrid.setCheckMissedContacts(true);
    double tsShare = 0.0;
    long steps = 0;
    while (!hybrid.getParticles().empty()) {
        hybrid.step();
        if (!hybrid.getParticles().empty()) {
            tsShare += static_cast<double>(hybrid.getTimeDrivenCount()) / hybrid.getParticles().size();
            ++steps;
        }
    }
    char line[200];
    std::snprintf(line, sizeof(line),
                  "k=%d, N=%d, L=%.0f: contactos con agentes ED = %ld  (fraccion TS media %.2f, evacuacion %.2f s)",
                  k, n, side, hybrid.getMissedContacts(), tsShare / steps, hybrid.getEvacuationTime());
    report(hybrid.getMissedContacts() == 0, line);
}

// Cuánto coincide el híbrido con el TSDMD tick a tick (posiciones y radios).
void divergence(int n, double side, unsigned seed, int k) {
    const types::Board board(side, 0.8);
    simulation::Simulation ts(n, seed, board);
    simulation::SimulationHybrid hybrid(n, seed, board, k);
    const double thresholds[] = {1e-12, 1e-9, 1e-6, 1e-3};
    double first[4] = {-1.0, -1.0, -1.0, -1.0};
    while (!ts.getParticles().empty() && !hybrid.getParticles().empty()) {
        ts.step();
        hybrid.step();
        const double d = maxDifference(ts.getParticles(), hybrid.getParticles(), false);
        for (int m = 0; m < 4; ++m) {
            if (first[m] < 0 && d > thresholds[m]) {
                first[m] = ts.getTime();
            }
        }
        if (first[3] >= 0) {
            break;
        }
    }
    std::printf("        k=%d, N=%d: diferencia con el TSDMD > 1e-12 desde t=%.2f s, > 1e-9 desde %.2f s, "
                "> 1e-6 desde %.2f s, > 1e-3 desde %.2f s\n",
                k, n, first[0], first[1], first[2], first[3]);
}

int check() {
    std::printf("[1] Todos en TS: debe ser identico byte a byte al TSDMD\n");
    checkAllTimeDriven(60, 12.0, 3);
    checkAllTimeDriven(500, 50.0, 42);

    std::printf("\n[2] Ningun agente ED toca a nadie (prediccion + cierre de la bolsa)\n");
    const int ks[] = {0, 1, 2, 4, 8};
    for (int k : ks) {
        checkNoMissedContacts(200, 50.0, 7, k);
    }
    checkNoMissedContacts(500, 50.0, 42, 1);
    checkNoMissedContacts(150, 10.0, 3, 1);

    std::printf("\n[3] Coincidencia tick a tick con el TSDMD (hasta la divergencia caotica)\n");
    divergence(200, 50.0, 7, 1);
    divergence(500, 50.0, 42, 1);
    divergence(500, 50.0, 42, 8);
    std::printf("        referencia: TSDMD contra si mismo con un agente movido 1e-12 m no se puede medir\n"
                "        con la interfaz publica; la comparacion estadistica esta en 'stats'.\n");
    std::printf("\nFallos: %d\n", failures);
    return failures == 0 ? 0 : 1;
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

int stats() {
    const int realizations = 10;
    const types::Board board(50.0, 0.8);
    std::vector<double> evTs;
    std::vector<double> evHy;
    std::printf("Hibrido (k=1) vs TSDMD: N=500, L=50, %d semillas\n", realizations);
    for (int r = 0; r < realizations; ++r) {
        const unsigned seed = 100u + static_cast<unsigned>(r);
        simulation::Simulation ts(500, seed, board);
        while (!ts.getParticles().empty()) {
            ts.step();
        }
        simulation::SimulationHybrid hybrid(500, seed, board, 1);
        while (!hybrid.getParticles().empty()) {
            hybrid.step();
        }
        evTs.push_back(ts.getEvacuationTime());
        evHy.push_back(hybrid.getEvacuationTime());
        std::printf("  semilla %u: TSDMD %.2f s   hibrido %.2f s\n", seed, ts.getEvacuationTime(),
                    hybrid.getEvacuationTime());
    }
    double mTs = 0.0;
    double sTs = 0.0;
    double mHy = 0.0;
    double sHy = 0.0;
    meanAndDeviation(evTs, mTs, sTs);
    meanAndDeviation(evHy, mHy, sHy);
    const double se = std::sqrt((sTs * sTs + sHy * sHy) / realizations);
    std::printf("  TSDMD   media %.2f s, desvio %.2f s\n", mTs, sTs);
    std::printf("  hibrido media %.2f s, desvio %.2f s\n", mHy, sHy);
    report(std::abs(mHy - mTs) < 3 * se, "medias compatibles: diferencia " + std::to_string(mHy - mTs) +
                                             " s, error estandar " + std::to_string(se) + " s");
    return failures == 0 ? 0 : 1;
}

int cost() {
    const types::Board board(50.0, 0.8);
    std::printf("Costo segun k: N=500, L=50, semilla 100 (sin escribir archivos)\n");
    double start = now();
    simulation::Simulation ts(500, 100, board);
    while (!ts.getParticles().empty()) {
        ts.step();
    }
    std::printf("  TSDMD:          %6.2f s de computo  evacuacion %.2f s\n", now() - start, ts.getEvacuationTime());
    start = now();
    simulation::SimulationEDMD ed(500, 100, board);
    while (!ed.getParticles().empty()) {
        ed.step();
    }
    std::printf("  EDMD (grilla):  %6.2f s de computo  evacuacion %.2f s\n", now() - start, ed.getEvacuationTime());
    const int ks[] = {0, 1, 2, 4, 8, 16, 32};
    for (int k : ks) {
        start = now();
        simulation::SimulationHybrid hybrid(500, 100, board, k);
        double tsShare = 0.0;
        long steps = 0;
        while (!hybrid.getParticles().empty()) {
            hybrid.step();
            if (!hybrid.getParticles().empty()) {
                tsShare += static_cast<double>(hybrid.getTimeDrivenCount()) / hybrid.getParticles().size();
                ++steps;
            }
        }
        const double seconds = now() - start;
        std::printf("  hibrido k=%-3d   %6.2f s de computo  evacuacion %.2f s  fraccion TS %.2f  "
                    "entradas por prediccion %ld  por cierre %ld  predicciones de pares %ld\n",
                    k, seconds, hybrid.getEvacuationTime(), tsShare / steps, hybrid.getPredictedEntries(),
                    hybrid.getBoundaryEntries(), hybrid.getPairPredictions());
    }
    return 0;
}

int scaling() {
    std::printf("Escalamiento a densidad 0,2 /m2, semilla 100, k=1 (sin escribir archivos)\n");
    std::printf("  %6s %8s %12s %12s %12s %12s\n", "N", "L [m]", "TS comp [s]", "HY comp [s]", "TS evac [s]", "HY evac [s]");
    const int sizes[] = {100, 200, 500, 1000, 2000};
    for (int n : sizes) {
        const double side = std::sqrt(n / 0.2);
        const types::Board board(side, 0.8);
        double start = now();
        simulation::Simulation ts(n, 100, board);
        while (!ts.getParticles().empty()) {
            ts.step();
        }
        const double tsSeconds = now() - start;
        start = now();
        simulation::SimulationHybrid hybrid(n, 100, board, 1);
        while (!hybrid.getParticles().empty()) {
            hybrid.step();
        }
        const double hySeconds = now() - start;
        std::printf("  %6d %8.1f %12.2f %12.2f %12.2f %12.2f\n", n, side, tsSeconds, hySeconds,
                    ts.getEvacuationTime(), hybrid.getEvacuationTime());
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
    if (mode == "cost") {
        return cost();
    }
    if (mode == "scaling") {
        return scaling();
    }
    std::printf("Modos: check, stats, cost, scaling\n");
    return 1;
}
