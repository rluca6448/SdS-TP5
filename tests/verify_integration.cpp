// Verificación del EDMD integrado (SimulationEDMD) contra el TSDMD y el prototipo.
#include "simulation/Simulation.h"
#include "simulation/SimulationEDMD.h"
#include "edmd_proto.h"

#include <chrono>
#include <cstdio>
#include <map>
#include <vector>

static int failures = 0;

static void report(bool ok, const char* name) {
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) {
        ++failures;
    }
}

// 1) Mismas posiciones iniciales en los dos esquemas.
static void testSameInitialState() {
    std::printf("\n[A] Estado inicial TSDMD vs EDMD (N=500, L=50, semilla 42)\n");
    const types::Board board(50.0, 0.8);
    simulation::Simulation ts(500, 42, board);
    simulation::SimulationEDMD ed(500, 42, board);
    double worst = 0.0;
    bool sameIds = ts.getParticles().size() == ed.getParticles().size();
    for (std::size_t k = 0; sameIds && k < ts.getParticles().size(); ++k) {
        const types::Particle& a = ts.getParticles()[k];
        const types::Particle& b = ed.getParticles()[k];
        sameIds = sameIds && a.getId() == b.getId();
        worst = std::max(worst, (a.getPosition() - b.getPosition()).norm());
        worst = std::max(worst, (a.getVelocity() - b.getVelocity()).norm());
        worst = std::max(worst, std::abs(a.getRadius() - b.getRadius()));
    }
    std::printf("        diferencia maxima (posicion, velocidad, radio) = %.3e\n", worst);
    report(sameIds && worst == 0.0, "posiciones, velocidades y radios identicos");
}

// 2) EDMD integrado vs prototipo: posiciones a tiempos comunes.
static void testMatchesPrototype(int n, double side, unsigned seed) {
    std::printf("\n[B] EDMD integrado vs prototipo (N=%d, L=%.0f, semilla %u)\n", n, side, seed);
    const types::Board board(side, 0.8);
    simulation::SimulationEDMD ed(n, seed, board);

    ::Board protoBoard = makeBoard(side, 0.8);
    protoBoard.exitTarget = ::Vector2D(0.5 * side, -1.0);   // mismo target exterior que el proyecto
    protoBoard.removeY = -1.0;
    EventDrivenSim proto(protoBoard);
    proto.checkInvariants = true;
    for (const types::Particle& p : ed.getParticles()) {
        proto.addAgent(p.getId(), ::Vector2D(p.getPosition().x, p.getPosition().y),
                       protoBoard.doorTarget, false);
    }

    // El prototipo no puede "espiar" su próximo evento: se avanza hasta pasarse de t
    // y se usa el estado previo al último grupo (válido en [tiempo previo, now)).
    // Ese estado se conserva entre instantes de muestreo: si un solo grupo salta
    // varios instantes, el estado previo sigue siendo el correcto.
    const double times[] = {0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0};
    std::vector<Agent> before = proto.agents;
    for (double t : times) {
        ed.advanceTo(t);
        while (proto.now <= t) {
            before = proto.agents;
            if (!proto.step()) {
                break;
            }
        }
        std::map<int, ::Vector2D> protoPositions;
        for (const Agent& a : before) {
            if (a.active) {
                protoPositions[a.id] = a.tr.positionAt(t);
            }
        }
        double worst = 0.0;
        int missing = 0;
        for (const types::Particle& p : ed.getParticles()) {
            std::map<int, ::Vector2D>::const_iterator it = protoPositions.find(p.getId());
            if (it == protoPositions.end()) {
                ++missing;
                continue;
            }
            const double dx = p.getPosition().x - it->second.x;
            const double dy = p.getPosition().y - it->second.y;
            worst = std::max(worst, std::sqrt(dx * dx + dy * dy));
        }
        std::printf("        t=%5.1f s  activos=%zu/%zu  max |dx|=%.3e m  sin pareja=%d\n", t,
                    ed.getParticles().size(), protoPositions.size(), worst, missing);
    }
    std::printf("        prototipo: solapamientos invalidos=%ld  fantasmas=%ld\n",
                proto.overlapViolations, proto.phantomContacts);
    report(proto.overlapViolations == 0, "prototipo sin solapamientos invalidos");
    report(ed.getPhantomContacts() == 0, "EDMD integrado sin contactos fantasma");
}

// 3) Comparación física TSDMD vs EDMD sobre varias semillas.
static void compareSchemes(int n, double side, int realizations) {
    std::printf("\n[C] TSDMD vs EDMD: N=%d, L=%.0f, %d semillas\n", n, side, realizations);
    const types::Board board(side, 0.8);
    std::vector<double> evTs;
    std::vector<double> evEd;
    std::vector<double> endTs;
    std::vector<double> endEd;
    double wallTs = 0.0;
    double wallEd = 0.0;
    for (int r = 0; r < realizations; ++r) {
        const unsigned seed = 100u + static_cast<unsigned>(r);
        std::chrono::steady_clock::time_point a0 = std::chrono::steady_clock::now();
        simulation::Simulation ts(n, seed, board);
        while (!ts.getParticles().empty()) {
            ts.step();
        }
        std::chrono::steady_clock::time_point a1 = std::chrono::steady_clock::now();
        simulation::SimulationEDMD ed(n, seed, board);
        while (!ed.getParticles().empty()) {
            ed.step();
        }
        std::chrono::steady_clock::time_point a2 = std::chrono::steady_clock::now();
        wallTs += std::chrono::duration<double>(a1 - a0).count();
        wallEd += std::chrono::duration<double>(a2 - a1).count();
        evTs.push_back(ts.getEvacuationTime());
        evEd.push_back(ed.getEvacuationTime());
        endTs.push_back(ts.getTime());
        endEd.push_back(ed.getTime());
        std::printf("        semilla %u: evacuacion TS=%.2f s  ED=%.2f s\n", seed,
                    ts.getEvacuationTime(), ed.getEvacuationTime());
    }
    double mTs = 0.0;
    double mEd = 0.0;
    for (int r = 0; r < realizations; ++r) {
        mTs += evTs[r] / realizations;
        mEd += evEd[r] / realizations;
    }
    double vTs = 0.0;
    double vEd = 0.0;
    for (int r = 0; r < realizations; ++r) {
        vTs += (evTs[r] - mTs) * (evTs[r] - mTs) / (realizations - 1);
        vEd += (evEd[r] - mEd) * (evEd[r] - mEd) / (realizations - 1);
    }
    const double se = std::sqrt(vTs / realizations + vEd / realizations);
    std::printf("        TSDMD: media=%.2f s  desvio=%.2f s  wall-clock medio=%.2f s\n",
                mTs, std::sqrt(vTs), wallTs / realizations);
    std::printf("        EDMD:  media=%.2f s  desvio=%.2f s  wall-clock medio=%.2f s\n",
                mEd, std::sqrt(vEd), wallEd / realizations);
    std::printf("        diferencia ED-TS = %.2f s  (error estandar %.2f s, %.1f errores)\n",
                mEd - mTs, se, (mEd - mTs) / se);
}

int main() {
    testSameInitialState();
    testMatchesPrototype(60, 12.0, 3u);
    testMatchesPrototype(200, 50.0, 7u);
    compareSchemes(200, 50.0, 10);
    compareSchemes(500, 50.0, 4);
    std::printf("\nFallos: %d\n", failures);
    return failures == 0 ? 0 : 1;
}
