// Tests simples del prototipo EDMD (alternativa B).
// Compilar: g++ -std=c++17 -O2 -Wall -Wextra test_edmd.cpp -o test_edmd

#include "edmd_proto.h"

#include <chrono>
#include <cstdio>
#include <random>

static int failures = 0;
static int passes = 0;

static void check(bool ok, const char* name, double got, double expected) {
    if (ok) {
        passes = passes + 1;
        std::printf("  PASS  %-48s got=%.9f expected=%.9f\n", name, got, expected);
    } else {
        failures = failures + 1;
        std::printf("  FAIL  %-48s got=%.9f expected=%.9f\n", name, got, expected);
    }
}

static void checkTrue(bool ok, const char* name) {
    if (ok) {
        passes = passes + 1;
        std::printf("  PASS  %s\n", name);
    } else {
        failures = failures + 1;
        std::printf("  FAIL  %s\n", name);
    }
}

static Trajectory makeTraj(Phase phase, const Vector2D& x, const Vector2D& dir,
                           double t0, double tRec0) {
    Trajectory tr;
    tr.phase = phase;
    tr.t0 = t0;
    tr.x0 = x;
    tr.dir = normalizedOrZero(dir);
    tr.tRec0 = tRec0;
    tr.version = 0;
    return tr;
}

static Trajectory freeTraj(const Vector2D& x, const Vector2D& dir) {
    return makeTraj(Phase::Free, x, dir, 0.0, 0.0);
}

// ---------------------------------------------------------------------------
static void testPairQuadratic() {
    std::printf("\n[1] Contacto agente-agente, casos analiticos\n");

    Trajectory a = freeTraj(Vector2D(0.0, 0.0), Vector2D(1.0, 0.0));
    Trajectory b = freeTraj(Vector2D(2.0, 0.0), Vector2D(-1.0, 0.0));
    ContactPrediction c = predictContact(a, b, 0.0, INF);
    check(c.found && std::fabs(c.time - 0.5) < 1e-9, "frente a frente", c.time, 0.5);

    Trajectory p1 = freeTraj(Vector2D(0.0, 0.0), Vector2D(1.0, 0.0));
    Trajectory p2 = freeTraj(Vector2D(0.0, 1.0), Vector2D(1.0, 0.0));
    c = predictContact(p1, p2, 0.0, INF);
    checkTrue(!c.found, "misma velocidad en paralelo: sin contacto");

    Trajectory s1 = freeTraj(Vector2D(0.0, 0.0), Vector2D(-1.0, 0.0));
    Trajectory s2 = freeTraj(Vector2D(2.0, 0.0), Vector2D(1.0, 0.0));
    c = predictContact(s1, s2, 0.0, INF);
    checkTrue(!c.found, "se alejan: sin contacto");

    Trajectory m1 = freeTraj(Vector2D(0.0, 0.0), Vector2D(1.0, 0.0));
    Trajectory m2 = freeTraj(Vector2D(2.0, 1.0), Vector2D(-1.0, 0.0));
    c = predictContact(m1, m2, 0.0, INF);
    checkTrue(!c.found, "cruce con desplazamiento lateral 1 m: sin contacto");

    // Desplazamiento lateral 0,4 m: se tocan cuando dx = 0,3 -> 2 - 3h = 0,3.
    Trajectory o1 = freeTraj(Vector2D(0.0, 0.0), Vector2D(1.0, 0.0));
    Trajectory o2 = freeTraj(Vector2D(2.0, 0.4), Vector2D(-1.0, 0.0));
    c = predictContact(o1, o2, 0.0, INF);
    double expected = 1.7 / 3.0;
    check(c.found && std::fabs(c.time - expected) < 1e-9, "choque oblicuo (lateral 0,4 m)",
          c.time, expected);

    Trajectory t1 = freeTraj(Vector2D(0.0, 0.0), Vector2D(1.0, 0.0));
    Trajectory t2 = freeTraj(Vector2D(0.4, 0.0), Vector2D(-1.0, 0.0));
    c = predictContact(t1, t2, 0.0, INF);
    checkTrue(!c.found, "ya solapados: no agenda evento (limitacion 1)");

    c = predictContact(a, b, 0.0, 0.4);
    checkTrue(!c.found, "choque despues del horizonte: no cuenta");

    // Escape cancelado (quieto) contra uno que viene.
    Trajectory q = makeTraj(Phase::Escape, Vector2D(0.0, 0.0), Vector2D(0.0, 0.0), 0.0, 0.0);
    Trajectory w = freeTraj(Vector2D(1.0, 0.0), Vector2D(-1.0, 0.0));
    c = predictContact(q, w, 0.0, INF);
    double exp2 = (1.0 - 0.35) / 1.5;
    check(c.found && std::fabs(c.time - exp2) < 1e-9, "escape quieto contra libre", c.time, exp2);
}

// ---------------------------------------------------------------------------
static void testRecoveryExample() {
    std::printf("\n[2] Recuperacion: ejemplo del chat (contacto a s ~ 0,194 s)\n");
    // A en 0,70 hacia +x, B en 1,30 hacia -x, ambos empiezan a recuperarse en t = 0.
    Trajectory a = makeTraj(Phase::Recovery, Vector2D(0.70, 0.0), Vector2D(1.0, 0.0), 0.0, 0.0);
    Trajectory b = makeTraj(Phase::Recovery, Vector2D(1.30, 0.0), Vector2D(-1.0, 0.0), 0.0, 0.0);
    ContactPrediction c = predictContact(a, b, 0.0, T_REC);

    // Referencia independiente: bisección de 0,4 - 2 D(s) - s = 0 en [0, 0,3].
    double lo = 0.0;
    double hi = 0.3;
    for (int it = 0; it < 200; ++it) {
        double mid = 0.5 * (lo + hi);
        double f = 0.6 - 2.0 * recoveryDistance(mid) - 2.0 * (RMIN + K * mid);
        if (f > 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    check(c.found && std::fabs(c.time - hi) < 1e-8, "contacto en recuperacion", c.time, hi);
    double r = a.radiusAt(c.time);
    check(std::fabs(r - (RMIN + K * hi)) < 1e-8, "radio en el contacto", r, RMIN + K * hi);
}

// ---------------------------------------------------------------------------
static void testPredictorsAgree() {
    std::printf("\n[3] Cuadratica vs avance conservador (5000 pares lineales al azar)\n");
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> pos(-1.5, 1.5);
    std::uniform_real_distribution<double> ang(0.0, 6.283185307179586);
    std::uniform_int_distribution<int> ph(0, 1);
    int mismatches = 0;
    int grazing = 0;
    int contacts = 0;
    double worstDt = 0.0;
    for (int k = 0; k < 5000; ++k) {
        Phase pa = ph(rng) == 0 ? Phase::Free : Phase::Escape;
        Phase pb = ph(rng) == 0 ? Phase::Free : Phase::Escape;
        double aa = ang(rng);
        double ab = ang(rng);
        Trajectory a = makeTraj(pa, Vector2D(pos(rng), pos(rng)),
                                Vector2D(std::cos(aa), std::sin(aa)), 0.0, 0.0);
        Trajectory b = makeTraj(pb, Vector2D(pos(rng), pos(rng)),
                                Vector2D(std::cos(ab), std::sin(ab)), 0.0, 0.0);
        double tEnd = 2.0;
        ContactPrediction q = nextContactQuadratic(a, b, 0.0, tEnd);
        ContactPrediction s = nextContactConservative(a, b, 0.0, tEnd);
        if (q.found && s.found) {
            contacts = contacts + 1;
            double dt = std::fabs(q.time - s.time);
            worstDt = std::max(worstDt, dt);
            if (dt > 1e-6) {
                mismatches = mismatches + 1;
            }
        } else if (q.found != s.found) {
            // Aceptable solo si es un roce más superficial que DEPTH_TOL.
            double minGap = INF;
            for (int m = 0; m <= 20000; ++m) {
                double t = tEnd * m / 20000.0;
                minGap = std::min(minGap, gapAt(a, b, t));
            }
            if (minGap > -2.0 * DEPTH_TOL) {
                grazing = grazing + 1;
            } else {
                mismatches = mismatches + 1;
            }
        }
    }
    std::printf("        contactos=%d  roces tolerados=%d  peor diferencia=%.3e s\n",
                contacts, grazing, worstDt);
    check(mismatches == 0, "sin discrepancias", mismatches, 0);
}

// ---------------------------------------------------------------------------
static void testRecoveryVsSampling() {
    std::printf("\n[4] Recuperacion vs muestreo denso (3000 pares al azar)\n");
    std::mt19937 rng(777);
    std::uniform_real_distribution<double> pos(-1.0, 1.0);
    std::uniform_real_distribution<double> ang(0.0, 6.283185307179586);
    std::uniform_real_distribution<double> frac(0.0, 0.9);
    std::uniform_int_distribution<int> ph(0, 2);
    int bad = 0;
    int contacts = 0;
    for (int k = 0; k < 3000; ++k) {
        double aa = ang(rng);
        double ab = ang(rng);
        // a siempre en recuperación (empezada hace un rato); b en cualquier fase.
        double ta = -frac(rng) * T_REC;
        Trajectory a = makeTraj(Phase::Recovery, Vector2D(pos(rng), pos(rng)),
                                Vector2D(std::cos(aa), std::sin(aa)), 0.0, ta);
        int pb = ph(rng);
        Phase phaseB = pb == 0 ? Phase::Free : (pb == 1 ? Phase::Escape : Phase::Recovery);
        double tb = -frac(rng) * T_REC;
        Trajectory b = makeTraj(phaseB, Vector2D(pos(rng), pos(rng)),
                                Vector2D(std::cos(ab), std::sin(ab)), 0.0, tb);
        double tEnd = std::min(a.phaseEnd(), b.phaseEnd());
        if (tEnd <= 0.0) {
            continue;
        }
        ContactPrediction c = predictContact(a, b, 0.0, tEnd);
        if (gapAt(a, b, 0.0) <= CONTACT_EPS) {
            if (c.found) {
                bad = bad + 1;
            }
            continue;
        }
        // Primer instante muestreado con solapamiento real (> DEPTH_TOL).
        double tSample = INF;
        const int samples = 30000;
        for (int m = 0; m <= samples; ++m) {
            double t = tEnd * m / samples;
            if (gapAt(a, b, t) < -DEPTH_TOL) {
                tSample = t;
                break;
            }
        }
        if (c.found) {
            contacts = contacts + 1;
            // Debe estar en contacto, y no después del primer solapamiento muestreado.
            if (gapAt(a, b, c.time) > CONTACT_EPS || c.time > tSample + 1e-12) {
                bad = bad + 1;
            }
        } else if (tSample != INF) {
            bad = bad + 1;          // se perdió un contacto real
        }
    }
    std::printf("        contactos encontrados=%d\n", contacts);
    check(bad == 0, "sin contactos perdidos ni tardios", bad, 0);
}

// ---------------------------------------------------------------------------
static void testWalls() {
    std::printf("\n[5] Paredes y puerta, casos analiticos (recinto 50 m, puerta 0,8 m)\n");
    Board board = makeBoard(50.0, 0.8);
    const Segment& top = board.walls[2];
    const Segment& bottomLeft = board.walls[3];
    const Segment& bottomRight = board.walls[4];

    Trajectory up = freeTraj(Vector2D(25.0, 49.0), Vector2D(0.0, 1.0));
    ContactPrediction c = predictWallContact(up, top, 0.0, INF);
    check(c.found && std::fabs(c.time - 0.5) < 1e-9, "choque normal contra techo", c.time, 0.5);

    Trajectory diag = freeTraj(Vector2D(20.0, 49.0), Vector2D(1.0, 1.0));
    c = predictWallContact(diag, top, 0.0, INF);
    double expDiag = 0.75 * std::sqrt(2.0) / 1.5;
    check(c.found && std::fabs(c.time - expDiag) < 1e-9, "choque oblicuo a 45 grados",
          c.time, expDiag);

    // Baja por x = doorLeft + 0,1: toca el extremo de la pared izquierda de la puerta.
    double x = board.doorLeft + 0.1;
    Trajectory edge = freeTraj(Vector2D(x, 1.0), Vector2D(0.0, -1.0));
    ContactPrediction cl = predictWallContact(edge, bottomLeft, 0.0, INF);
    ContactPrediction cr = predictWallContact(edge, bottomRight, 0.0, INF);
    double yc = std::sqrt(0.25 * 0.25 - 0.1 * 0.1);
    double expEdge = (1.0 - yc) / 1.5;
    check(cl.found && std::fabs(cl.time - expEdge) < 1e-9, "extremo de puerta (izquierdo)",
          cl.time, expEdge);
    checkTrue(!cr.found, "no toca el extremo derecho (0,6 m de distancia)");

    // Por el centro de la puerta pasa sin tocar nada.
    Trajectory center = freeTraj(Vector2D(25.0, 1.0), Vector2D(0.0, -1.0));
    bool any = false;
    for (int w = 0; w < static_cast<int>(board.walls.size()); ++w) {
        if (predictWallContact(center, board.walls[w], 0.0, 3.0).found) {
            any = true;
        }
    }
    checkTrue(!any, "centro de la puerta: pasa sin contacto");

    // Desde afuera, por debajo de la pared inferior.
    Trajectory below = freeTraj(Vector2D(10.0, -1.0), Vector2D(0.0, 1.0));
    c = predictWallContact(below, bottomLeft, 0.0, INF);
    check(c.found && std::fabs(c.time - 0.75 / 1.5) < 1e-9, "contacto desde el exterior",
          c.time, 0.5);

    // En recuperación contra el techo: referencia por bisección.
    Trajectory rec = makeTraj(Phase::Recovery, Vector2D(25.0, 49.7), Vector2D(0.0, 1.0), 0.0, 0.0);
    c = predictWallContact(rec, top, 0.0, T_REC);
    double lo = 0.0;
    double hi = T_REC;
    for (int it = 0; it < 200; ++it) {
        double mid = 0.5 * (lo + hi);
        double f = 0.3 - recoveryDistance(mid) - (RMIN + K * mid);
        if (f > 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    check(c.found && std::fabs(c.time - hi) < 1e-8, "pared en recuperacion", c.time, hi);
}

// ---------------------------------------------------------------------------
static void testWallPredictorsAgree() {
    std::printf("\n[6] Pared: analitico vs avance conservador (5000 casos al azar)\n");
    Board board = makeBoard(5.0, 0.8);
    std::mt19937 rng(99);
    std::uniform_real_distribution<double> px(-1.0, 6.0);
    std::uniform_real_distribution<double> ang(0.0, 6.283185307179586);
    int mismatches = 0;
    int grazing = 0;
    int contacts = 0;
    for (int k = 0; k < 5000; ++k) {
        double a = ang(rng);
        Trajectory tr = freeTraj(Vector2D(px(rng), px(rng)), Vector2D(std::cos(a), std::sin(a)));
        for (int w = 0; w < static_cast<int>(board.walls.size()); ++w) {
            const Segment& s = board.walls[w];
            if (wallGapAt(tr, s, 0.0) <= CONTACT_EPS) {
                continue;
            }
            ContactPrediction an = nextWallContactLinear(tr, s, 0.0, 3.0);
            ContactPrediction co = nextWallContactConservative(tr, s, 0.0, 3.0);
            if (an.found && co.found) {
                contacts = contacts + 1;
                if (std::fabs(an.time - co.time) > 1e-6) {
                    mismatches = mismatches + 1;
                }
            } else if (an.found != co.found) {
                double minGap = INF;
                for (int m = 0; m <= 30000; ++m) {
                    minGap = std::min(minGap, wallGapAt(tr, s, 3.0 * m / 30000.0));
                }
                if (minGap > -2.0 * DEPTH_TOL) {
                    grazing = grazing + 1;
                } else {
                    mismatches = mismatches + 1;
                }
            }
        }
    }
    std::printf("        contactos=%d  roces tolerados=%d\n", contacts, grazing);
    check(mismatches == 0, "sin discrepancias", mismatches, 0);
}

// ---------------------------------------------------------------------------
static void testHeadOnSimulation() {
    std::printf("\n[7] Simulacion: dos agentes frente a frente (ciclo contacto-escape-recuperacion)\n");
    BruteForceEdSim sim(makeBoard(50.0, 0.8));
    sim.keepLog = true;
    sim.addAgent(0, Vector2D(24.0, 25.0), Vector2D(40.0, 25.0), true);
    sim.addAgent(1, Vector2D(26.0, 25.0), Vector2D(10.0, 25.0), true);
    std::vector<double> contactTimes;
    while (sim.now < 1.0 && sim.step()) {
    }
    for (int k = 0; k < static_cast<int>(sim.log.size()); ++k) {
        if (sim.log[k].type == EventType::PairContact) {
            contactTimes.push_back(sim.log[k].time);
        }
    }
    double lo = 0.0;
    double hi = 0.3;
    for (int it = 0; it < 200; ++it) {
        double mid = 0.5 * (lo + hi);
        double f = 0.6 - 2.0 * recoveryDistance(mid) - 2.0 * (RMIN + K * mid);
        if (f > 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    double expected2 = 0.5 + T_ESC + hi;
    checkTrue(contactTimes.size() >= 2, "hay al menos dos contactos");
    if (contactTimes.size() >= 2) {
        check(std::fabs(contactTimes[0] - 0.5) < 1e-9, "primer contacto", contactTimes[0], 0.5);
        check(std::fabs(contactTimes[1] - expected2) < 1e-8, "segundo contacto",
              contactTimes[1], expected2);
    }
    check(sim.overlapViolations == 0, "sin solapamientos invalidos", sim.overlapViolations, 0);
    check(sim.phantomContacts == 0, "sin contactos fantasma", sim.phantomContacts, 0);
}

// ---------------------------------------------------------------------------
static void testSingleEvacuation() {
    std::printf("\n[8] Simulacion: un agente sale por la puerta\n");
    BruteForceEdSim sim(makeBoard(50.0, 0.8));
    sim.keepLog = true;
    Board b = sim.board;
    sim.addAgent(0, Vector2D(25.0, 5.0), b.doorTarget, false);
    while (sim.step()) {
    }
    double tDoor = INF;
    double tRemove = INF;
    for (int k = 0; k < static_cast<int>(sim.log.size()); ++k) {
        if (sim.log[k].type == EventType::DoorCross) {
            tDoor = sim.log[k].time;
        }
        if (sim.log[k].type == EventType::Remove) {
            tRemove = sim.log[k].time;
        }
    }
    check(std::fabs(tDoor - 5.0 / 1.5) < 1e-9, "cruce de puerta", tDoor, 5.0 / 1.5);
    check(std::fabs(tRemove - 6.0 / 1.5) < 1e-9, "retiro en y = -1", tRemove, 4.0);
    checkTrue(sim.activeCount() == 0, "no quedan agentes activos");
}

// ---------------------------------------------------------------------------
static std::vector<Vector2D> runThree(const std::vector<int>& order) {
    BruteForceEdSim sim(makeBoard(50.0, 0.8));
    std::vector<Vector2D> start(3);
    std::vector<Vector2D> target(3);
    start[0] = Vector2D(25.0, 25.0); target[0] = Vector2D(25.0, 25.0);   // quieto
    start[1] = Vector2D(23.0, 25.0); target[1] = Vector2D(40.0, 25.0);
    start[2] = Vector2D(27.0, 25.0); target[2] = Vector2D(10.0, 25.0);
    for (int k = 0; k < 3; ++k) {
        int id = order[k];
        sim.addAgent(id, start[id], target[id], true);
    }
    while (sim.now < 1.5 && sim.step()) {
    }
    std::vector<Vector2D> result(3);
    for (int k = 0; k < 3; ++k) {
        result[sim.agents[k].id] = sim.agents[k].tr.positionAt(1.5);
    }
    return result;
}

static void testThreeSymmetric() {
    std::printf("\n[9] Simulacion: tres agentes, contacto simultaneo y permutacion\n");
    std::vector<int> o1 = {0, 1, 2};
    std::vector<int> o2 = {2, 0, 1};
    std::vector<Vector2D> r1 = runThree(o1);
    std::vector<Vector2D> r2 = runThree(o2);
    double diff = 0.0;
    for (int k = 0; k < 3; ++k) {
        diff = std::max(diff, (r1[k] - r2[k]).norm());
    }
    check(diff < 1e-12, "mismo resultado al permutar el orden", diff, 0.0);
    double sym = std::fabs((r1[1].x - 25.0) + (r1[2].x - 25.0));
    check(sym < 1e-12, "B y C quedan simetricos respecto de A", sym, 0.0);
    double moved = (r1[0] - Vector2D(25.0, 25.0)).norm();
    check(moved < 1e-12, "A no se mueve (escape cancelado)", moved, 0.0);
}

// ---------------------------------------------------------------------------
static void testSmallEvacuation(int n, double side, unsigned seed) {
    std::printf("\n[10] Evacuacion: N=%d, recinto %.0f m, puerta 0,8 m, semilla %u\n", n, side, seed);
    BruteForceEdSim sim(makeBoard(side, 0.8));
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(RMAX + 0.05, side - RMAX - 0.05);
    std::vector<Vector2D> placed;
    int id = 0;
    int tries = 0;
    while (id < n && tries < 100000) {
        tries = tries + 1;
        Vector2D p(u(rng), u(rng));
        bool ok = true;
        for (int k = 0; k < static_cast<int>(placed.size()); ++k) {
            if ((p - placed[k]).norm() < 2.0 * RMAX + 0.01) {
                ok = false;
            }
        }
        if (ok) {
            placed.push_back(p);
            sim.addAgent(id, p, sim.board.doorTarget, false);
            id = id + 1;
        }
    }
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    bool finished = true;
    while (sim.activeCount() > 0) {
        if (!sim.step()) {
            finished = false;          // agenda vacía con activos
            break;
        }
        if (sim.now > 600.0 || sim.groups > 5000000) {
            finished = false;
            break;
        }
    }
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double wall = std::chrono::duration<double>(t1 - t0).count();
    std::printf("        tiempo fisico=%.3f s  grupos de eventos=%ld  wall-clock=%.2f s\n",
                sim.now, sim.groups, wall);
    std::printf("        peor solapamiento=%.3e m  fantasmas=%ld\n",
                sim.worstPenetration, sim.phantomContacts);
    checkTrue(finished && sim.activeCount() == 0, "todos evacuan");
    check(sim.overlapViolations == 0, "sin solapamientos invalidos", sim.overlapViolations, 0);
    check(sim.phantomContacts == 0, "sin contactos fantasma", sim.phantomContacts, 0);
}

// ---------------------------------------------------------------------------
// Agenda con heap vs fuerza bruta
// ---------------------------------------------------------------------------
static std::vector<Vector2D> randomPlacement(int n, double side, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(RMAX + 0.05, side - RMAX - 0.05);
    std::vector<Vector2D> placed;
    int tries = 0;
    while (static_cast<int>(placed.size()) < n && tries < 1000000) {
        tries = tries + 1;
        Vector2D p(u(rng), u(rng));
        bool ok = true;
        for (int k = 0; k < static_cast<int>(placed.size()); ++k) {
            if ((p - placed[k]).norm() < 2.0 * RMAX + 0.01) {
                ok = false;
                break;
            }
        }
        if (ok) {
            placed.push_back(p);
        }
    }
    return placed;
}

template <typename Sim>
static bool runToEnd(Sim& sim, double tMax) {
    while (sim.activeCount() > 0) {
        if (!sim.step()) {
            return false;
        }
        if (sim.now > tMax) {
            return false;
        }
    }
    return true;
}

// Clave de un evento que no depende del orden de los agentes dentro del par.
static long long eventKey(const Event& e) {
    int a = e.i;
    int b = e.j;
    if (e.type == EventType::PairContact && b < a) {
        std::swap(a, b);
    }
    return (static_cast<long long>(e.type) * 100000LL + a) * 100000LL + (b + 1);
}

static std::vector<long long> groupKeys(const std::vector<Event>& group) {
    std::vector<long long> keys;
    for (int k = 0; k < static_cast<int>(group.size()); ++k) {
        keys.push_back(eventKey(group[k]));
    }
    std::sort(keys.begin(), keys.end());
    return keys;
}

// Avanza dos simulaciones a la par y devuelve el primer grupo en que procesan
// eventos distintos (otro tipo u otros agentes). maxDt: la mayor diferencia
// de instante entre ellas antes de ese grupo.
template <typename SimA, typename SimB>
static long lockstepDivergence(SimA& a, SimB& b, double& maxDt, bool& bothEnded) {
    maxDt = 0.0;
    bothEnded = false;
    a.keepLog = true;
    b.keepLog = true;
    for (long g = 0; ; ++g) {
        a.log.clear();
        b.log.clear();
        bool okA = a.step();
        bool okB = b.step();
        if (!okA || !okB) {
            bothEnded = !okA && !okB;
            return g;
        }
        if (groupKeys(a.log) != groupKeys(b.log)) {
            return g;
        }
        maxDt = std::max(maxDt, std::fabs(a.now - b.now));
        if (a.activeCount() == 0 && b.activeCount() == 0) {
            bothEnded = true;
            return g + 1;
        }
    }
}

static void testHeapMatchesBrute(int n, double side, unsigned seed) {
    std::printf("\n[11] Agenda vs fuerza bruta: N=%d, recinto %.0f m, semilla %u\n", n, side, seed);
    std::vector<Vector2D> pos = randomPlacement(n, side, seed);
    Board board = makeBoard(side, 0.8);

    // Agenda contra fuerza bruta, mismas condiciones iniciales.
    BruteForceEdSim brute(board);
    EventDrivenSim heap(board);
    for (int k = 0; k < n; ++k) {
        brute.addAgent(k, pos[k], board.doorTarget, false);
        heap.addAgent(k, pos[k], board.doorTarget, false);
    }
    double dtHB = 0.0;
    bool endHB = false;
    long divHB = lockstepDivergence(brute, heap, dtHB, endHB);

    // Referencia de caos: fuerza bruta contra sí misma, un agente movido 1e-12 m.
    BruteForceEdSim b1(board);
    BruteForceEdSim b2(board);
    for (int k = 0; k < n; ++k) {
        b1.addAgent(k, pos[k], board.doorTarget, false);
        Vector2D p = pos[k];
        if (k == 0) {
            p = p + Vector2D(1e-12, 0.0);
        }
        b2.addAgent(k, p, board.doorTarget, false);
    }
    double dtBB = 0.0;
    bool endBB = false;
    long divBB = lockstepDivergence(b1, b2, dtBB, endBB);

    std::printf("        agenda vs bruta:            mismos eventos hasta el grupo %ld (max dt previo %.1e s)%s\n",
                divHB, dtHB, endHB ? " [hasta el final]" : "");
    std::printf("        bruta vs bruta + 1e-12 m:   mismos eventos hasta el grupo %ld (max dt previo %.1e s)%s\n",
                divBB, dtBB, endBB ? " [hasta el final]" : "");

    bool okH = runToEnd(heap, 600.0);
    std::printf("        agenda: evacuacion=%.3f s  agendados=%ld usados=%ld obsoletos=%ld\n",
                heap.now, heap.pushed, heap.popped, heap.stale);
    checkTrue(endHB || 4 * divHB >= divBB,
              "la agenda solo diverge al ritmo del caos (no antes)");
    checkTrue(okH, "la agenda evacua a todos");
    check(heap.overlapViolations == 0, "agenda sin solapamientos invalidos",
          heap.overlapViolations, 0);
    check(heap.phantomContacts == 0, "agenda sin contactos fantasma", heap.phantomContacts, 0);
}

// Como el sistema es caótico, la comparación final es estadística.
static void testHeapStatistics(int n, double side, int realizations) {
    std::printf("\n[11b] Estadistica de evacuacion: N=%d, recinto %.0f m, %d semillas\n",
                n, side, realizations);
    Board board = makeBoard(side, 0.8);
    std::vector<double> tb;
    std::vector<double> th;
    for (int r = 0; r < realizations; ++r) {
        std::vector<Vector2D> pos = randomPlacement(n, side, 1000u + static_cast<unsigned>(r));
        BruteForceEdSim brute(board);
        EventDrivenSim heap(board);
        brute.checkInvariants = false;
        heap.checkInvariants = false;
        for (int k = 0; k < n; ++k) {
            brute.addAgent(k, pos[k], board.doorTarget, false);
            heap.addAgent(k, pos[k], board.doorTarget, false);
        }
        runToEnd(brute, 1000.0);
        runToEnd(heap, 1000.0);
        tb.push_back(brute.now);
        th.push_back(heap.now);
    }
    double mb = 0.0;
    double mh = 0.0;
    for (int r = 0; r < realizations; ++r) {
        mb = mb + tb[r] / realizations;
        mh = mh + th[r] / realizations;
    }
    double vb = 0.0;
    double vh = 0.0;
    for (int r = 0; r < realizations; ++r) {
        vb = vb + (tb[r] - mb) * (tb[r] - mb) / (realizations - 1);
        vh = vh + (th[r] - mh) * (th[r] - mh) / (realizations - 1);
    }
    double se = std::sqrt(vb / realizations + vh / realizations);
    std::printf("        bruta:  media=%.3f s  desvio=%.3f s\n", mb, std::sqrt(vb));
    std::printf("        agenda: media=%.3f s  desvio=%.3f s\n", mh, std::sqrt(vh));
    std::printf("        diferencia de medias=%.3f s  (error estandar %.3f s)\n", mh - mb, se);
    checkTrue(std::fabs(mh - mb) < 3.0 * se, "medias compatibles (|dif| < 3 errores estandar)");
}

static void testHeapPerformance(int n, double side, unsigned seed, bool alsoBrute) {
    std::printf("\n[12] Rendimiento: N=%d, recinto %.0f m (densidad %.3f /m2), semilla %u\n",
                n, side, n / (side * side), seed);
    std::vector<Vector2D> pos = randomPlacement(n, side, seed);
    Board board = makeBoard(side, 0.8);
    EventDrivenSim heap(board);
    heap.checkInvariants = false;
    for (int k = 0; k < n; ++k) {
        heap.addAgent(k, pos[k], board.doorTarget, false);
    }
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    bool ok = runToEnd(heap, 3000.0);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double wall = std::chrono::duration<double>(t1 - t0).count();
    std::printf("        agenda: evacuacion=%.2f s  grupos=%ld  obsoletos=%.0f%%  wall-clock=%.2f s\n",
                heap.now, heap.groups,
                100.0 * heap.stale / static_cast<double>(heap.stale + heap.popped), wall);
    checkTrue(ok, "evacuan todos");

    if (alsoBrute) {
        BruteForceEdSim brute(board);
        brute.checkInvariants = false;
        for (int k = 0; k < n; ++k) {
            brute.addAgent(k, pos[k], board.doorTarget, false);
        }
        std::chrono::steady_clock::time_point b0 = std::chrono::steady_clock::now();
        runToEnd(brute, 3000.0);
        std::chrono::steady_clock::time_point b1 = std::chrono::steady_clock::now();
        double wallB = std::chrono::duration<double>(b1 - b0).count();
        std::printf("        fuerza bruta: wall-clock=%.2f s  (agenda %.1fx mas rapida)\n",
                    wallB, wallB / wall);
    }
}

int main() {
    std::printf("T_ESC=%.6f s  T_REC=%.6f s  K=%.3f m/s\n", T_ESC, T_REC, K);
    testPairQuadratic();
    testRecoveryExample();
    testPredictorsAgree();
    testRecoveryVsSampling();
    testWalls();
    testWallPredictorsAgree();
    testHeadOnSimulation();
    testSingleEvacuation();
    testThreeSymmetric();
    testSmallEvacuation(20, 6.0, 1u);
    testSmallEvacuation(20, 6.0, 2u);
    testSmallEvacuation(50, 8.0, 3u);
    testHeapMatchesBrute(20, 6.0, 1u);
    testHeapMatchesBrute(50, 8.0, 3u);
    testHeapMatchesBrute(100, 12.0, 4u);
    testHeapStatistics(30, 7.0, 12);
    testHeapPerformance(100, 50.0, 5u, true);
    testHeapPerformance(200, 50.0, 6u, false);
    testHeapPerformance(500, 50.0, 7u, false);
    std::printf("\nResultado: %d PASS, %d FAIL\n", passes, failures);
    return failures == 0 ? 0 : 1;
}
