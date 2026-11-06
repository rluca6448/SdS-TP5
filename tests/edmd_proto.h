#pragma once
// Prototipo de EDMD (alternativa B) para CPM.
// Independiente del proyecto: sirve para probar predictores y reglas
// antes de integrarlos en Simulation.

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

// ---------------------------------------------------------------------------
// Vector mínimo (en el proyecto se usa el Vector2D existente)
// ---------------------------------------------------------------------------
struct Vector2D {
    double x;
    double y;
    Vector2D() : x(0.0), y(0.0) {}
    Vector2D(double xValue, double yValue) : x(xValue), y(yValue) {}
    Vector2D operator+(const Vector2D& o) const { return Vector2D(x + o.x, y + o.y); }
    Vector2D operator-(const Vector2D& o) const { return Vector2D(x - o.x, y - o.y); }
    Vector2D operator*(double s) const { return Vector2D(x * s, y * s); }
    double dot(const Vector2D& o) const { return x * o.x + y * o.y; }
    double norm() const { return std::sqrt(x * x + y * y); }
};

inline Vector2D normalizedOrZero(const Vector2D& v) {
    double n = v.norm();
    if (n > 1e-12) {
        return v * (1.0 / n);
    }
    return Vector2D(0.0, 0.0);
}

// ---------------------------------------------------------------------------
// Parámetros CPM y tolerancias
// ---------------------------------------------------------------------------
const double RMIN = 0.10;                  // [m]
const double RMAX = 0.25;                  // [m]
const double VD = 1.5;                     // [m/s]
const double VE = 1.5;                     // [m/s]
const double TAU = 0.5;                    // [s]
const double BETA = 0.9;
const double K = RMAX / TAU;               // [m/s] crecimiento del radio
const double T_REC = (RMAX - RMIN) / K;    // [s] duración de la recuperación (0,3 s)
const double T_ESC = RMIN / (2.0 * VE);    // [s] duración del escape (= dt)

const double CONTACT_EPS = 1e-9;           // [m] hueco <= esto es contacto
const double DEPTH_TOL = 1e-6;             // [m] penetración máxima que se acepta no ver
const double TIME_TOL = 1e-12;             // [s] precisión de la bisección
const double GROUP_TOL = 1e-11;            // [s] eventos a menos de esto son simultáneos
const double INF = std::numeric_limits<double>::infinity();

// ---------------------------------------------------------------------------
// Trayectorias
// ---------------------------------------------------------------------------
enum class Phase { Free, Escape, Recovery };

// Distancia recorrida en recuperación, sigma = tiempo desde que el radio valía RMIN.
inline double recoveryDistance(double sigma) {
    if (sigma <= 0.0) {
        return 0.0;
    }
    if (sigma > T_REC) {
        sigma = T_REC;
    }
    return VD * T_REC / (1.0 + BETA) * std::pow(sigma / T_REC, 1.0 + BETA);
}

// Inversa de recoveryDistance: sigma para recorrer dist. INF si no alcanza en la fase.
inline double recoveryTimeForDistance(double dist) {
    double dMax = VD * T_REC / (1.0 + BETA);
    if (dist > dMax) {
        return INF;
    }
    if (dist <= 0.0) {
        return 0.0;
    }
    return T_REC * std::pow(dist / dMax, 1.0 / (1.0 + BETA));
}

struct Trajectory {
    Phase phase;
    double t0;        // inicio de este tramo
    Vector2D x0;      // posición en t0
    Vector2D dir;     // dirección unitaria (o cero)
    double tRec0;     // solo Recovery: instante en que el radio valía RMIN
    int version;

    double radiusAt(double t) const {
        if (phase == Phase::Free) {
            return RMAX;
        }
        if (phase == Phase::Escape) {
            return RMIN;
        }
        double r = RMIN + K * (t - tRec0);
        if (r > RMAX) {
            r = RMAX;
        }
        return r;
    }

    Vector2D positionAt(double t) const {
        if (phase == Phase::Free) {
            return x0 + dir * (VD * (t - t0));
        }
        if (phase == Phase::Escape) {
            return x0 + dir * (VE * (t - t0));
        }
        double d = recoveryDistance(t - tRec0) - recoveryDistance(t0 - tRec0);
        return x0 + dir * d;
    }

    double phaseEnd() const {
        if (phase == Phase::Escape) {
            return t0 + T_ESC;
        }
        if (phase == Phase::Recovery) {
            return tRec0 + T_REC;
        }
        return INF;
    }
};

inline Vector2D linearVelocity(const Trajectory& tr) {
    if (tr.phase == Phase::Free) {
        return tr.dir * VD;
    }
    return tr.dir * VE;
}

inline double maxSpeedUntil(const Trajectory& tr, double tEnd) {
    if (tr.dir.norm() == 0.0) {
        return 0.0;
    }
    if (tr.phase == Phase::Free) {
        return VD;
    }
    if (tr.phase == Phase::Escape) {
        return VE;
    }
    double s = (tEnd - tr.tRec0) / T_REC;
    if (s < 0.0) {
        s = 0.0;
    }
    if (s > 1.0) {
        s = 1.0;
    }
    return VD * std::pow(s, BETA);
}

inline double radiusGrowthRate(const Trajectory& tr) {
    if (tr.phase == Phase::Recovery) {
        return K;
    }
    return 0.0;
}

// ---------------------------------------------------------------------------
// Resultado de una predicción
// ---------------------------------------------------------------------------
struct ContactPrediction {
    bool found;
    double time;
};

inline ContactPrediction noContact() {
    ContactPrediction c;
    c.found = false;
    c.time = 0.0;
    return c;
}

inline ContactPrediction contactAt(double t) {
    ContactPrediction c;
    c.found = true;
    c.time = t;
    return c;
}

// ---------------------------------------------------------------------------
// Avance conservador genérico (limitaciones 1, 2 y 3)
// gap(t): hueco en t. L: cota de qué tan rápido puede achicarse.
// ---------------------------------------------------------------------------
inline ContactPrediction conservativeRoot(const std::function<double(double)>& gap,
                                          double L, double tStart, double tEnd) {
    double fPrev = gap(tStart);
    if (fPrev <= CONTACT_EPS) {
        return noContact();                // contacto existente
    }
    if (L <= 0.0 || tEnd <= tStart) {
        return noContact();
    }
    const double hMin = 2.0 * DEPTH_TOL / L;
    const int maxSteps = 1000000;

    // Mismo criterio que la cuadrática: el instante de contacto es donde el
    // hueco vale CONTACT_EPS/2 (así contactSetAt lo ve con margen).
    const double target = 0.5 * CONTACT_EPS;

    double tPrev = tStart;
    int steps = 0;
    while (tPrev < tEnd) {
        double h = (fPrev - target) / L;
        if (h < hMin) {
            h = hMin;
        }
        double tNext = tPrev + h;
        if (tNext > tEnd) {
            tNext = tEnd;
        }
        double fNext = gap(tNext);
        if (fNext <= target) {
            double lo = tPrev;
            double hi = tNext;
            for (int it = 0; it < 100 && hi - lo > TIME_TOL; ++it) {
                double mid = 0.5 * (lo + hi);
                if (gap(mid) <= target) {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            return contactAt(hi);
        }
        tPrev = tNext;
        fPrev = fNext;
        steps = steps + 1;
        if (steps > maxSteps) {
            throw std::runtime_error("conservativeRoot: no converge");
        }
    }
    return noContact();
}

// Primera entrada de un punto que se mueve (p + u*h) en un disco de radio R
// centrado en el origen. INF si no entra. Forma estable de la raíz menor.
inline double firstEntryPointDisc(const Vector2D& p, const Vector2D& u, double R) {
    double qa = u.dot(u);
    double qb = 2.0 * p.dot(u);
    double qc = p.dot(p) - R * R;
    if (qc <= 0.0) {
        return INF;                        // ya adentro: no es una entrada futura
    }
    if (qb >= 0.0) {
        return INF;                        // no se acerca
    }
    double disc = qb * qb - 4.0 * qa * qc;
    if (disc < 0.0) {
        return INF;                        // pasa sin entrar
    }
    double q = 0.5 * (-qb + std::sqrt(disc));
    return qc / q;
}

// ---------------------------------------------------------------------------
// Agente-agente
// ---------------------------------------------------------------------------
inline double gapAt(const Trajectory& a, const Trajectory& b, double t) {
    Vector2D d = a.positionAt(t) - b.positionAt(t);
    return d.norm() - (a.radiusAt(t) + b.radiusAt(t));
}

inline double gapClosingBound(const Trajectory& a, const Trajectory& b, double tEnd) {
    return maxSpeedUntil(a, tEnd) + maxSpeedUntil(b, tEnd)
         + radiusGrowthRate(a) + radiusGrowthRate(b);
}

inline ContactPrediction nextContactQuadratic(const Trajectory& a, const Trajectory& b,
                                              double tStart, double tEnd) {
    if (gapAt(a, b, tStart) <= CONTACT_EPS) {
        return noContact();
    }
    Vector2D p = a.positionAt(tStart) - b.positionAt(tStart);
    Vector2D u = linearVelocity(a) - linearVelocity(b);
    double R = a.radiusAt(tStart) + b.radiusAt(tStart) + 0.5 * CONTACT_EPS;
    double h = firstEntryPointDisc(p, u, R);
    if (h == INF || tStart + h > tEnd) {
        return noContact();
    }
    return contactAt(tStart + h);
}

inline ContactPrediction nextContactConservative(const Trajectory& a, const Trajectory& b,
                                                 double tStart, double tEnd) {
    std::function<double(double)> gap = [&a, &b](double t) { return gapAt(a, b, t); };
    return conservativeRoot(gap, gapClosingBound(a, b, tEnd), tStart, tEnd);
}

// tEnd lo decide quien llama: mínimo de los horizontes de validez de ambos.
inline ContactPrediction predictContact(const Trajectory& a, const Trajectory& b,
                                        double tStart, double tEnd) {
    // Filtro de alcance: si el hueco no se puede cerrar antes de tEnd, no hay choque.
    if (tEnd != INF) {
        double reach = gapClosingBound(a, b, tEnd) * (tEnd - tStart);
        if (gapAt(a, b, tStart) > reach + CONTACT_EPS) {
            return noContact();
        }
    }
    if (a.phase == Phase::Recovery || b.phase == Phase::Recovery) {
        return nextContactConservative(a, b, tStart, tEnd);
    }
    return nextContactQuadratic(a, b, tStart, tEnd);
}

// ---------------------------------------------------------------------------
// Agente-pared (segmentos finitos = cápsulas)
// ---------------------------------------------------------------------------
struct Segment {
    Vector2D a;
    Vector2D b;
};

inline Vector2D closestPoint(const Segment& s, const Vector2D& p) {
    Vector2D ab = s.b - s.a;
    double lambda = (p - s.a).dot(ab) / ab.dot(ab);
    if (lambda < 0.0) {
        lambda = 0.0;
    }
    if (lambda > 1.0) {
        lambda = 1.0;
    }
    return s.a + ab * lambda;
}

inline double wallGapAt(const Trajectory& tr, const Segment& s, double t) {
    Vector2D x = tr.positionAt(t);
    return (x - closestPoint(s, x)).norm() - tr.radiusAt(t);
}

// Movimiento rectilíneo y radio fijo: banda lateral + discos en los extremos.
inline ContactPrediction nextWallContactLinear(const Trajectory& tr, const Segment& s,
                                               double tStart, double tEnd) {
    if (wallGapAt(tr, s, tStart) <= CONTACT_EPS) {
        return noContact();
    }
    Vector2D p = tr.positionAt(tStart);
    Vector2D v = linearVelocity(tr);
    double rr = tr.radiusAt(tStart) + 0.5 * CONTACT_EPS;

    Vector2D ab = s.b - s.a;
    double len = ab.norm();
    Vector2D d = ab * (1.0 / len);          // a lo largo del segmento
    Vector2D n(-d.y, d.x);                  // normal

    double best = INF;

    // Banda: |distancia normal| llega a rr con la proyección dentro del segmento.
    double sn0 = n.dot(p - s.a);
    double vn = n.dot(v);
    if (std::fabs(sn0) > rr && vn != 0.0) {
        double side = 1.0;
        if (sn0 < 0.0) {
            side = -1.0;
        }
        double h = (side * rr - sn0) / vn;
        if (h >= 0.0) {
            double lambda = d.dot(p + v * h - s.a);
            if (lambda >= 0.0 && lambda <= len) {
                best = h;
            }
        }
    }

    // Extremos: igual que contra un disco quieto de radio 0.
    best = std::min(best, firstEntryPointDisc(p - s.a, v, rr));
    best = std::min(best, firstEntryPointDisc(p - s.b, v, rr));

    if (best == INF || tStart + best > tEnd) {
        return noContact();
    }
    return contactAt(tStart + best);
}

inline ContactPrediction nextWallContactConservative(const Trajectory& tr, const Segment& s,
                                                     double tStart, double tEnd) {
    std::function<double(double)> gap = [&tr, &s](double t) { return wallGapAt(tr, s, t); };
    double L = maxSpeedUntil(tr, tEnd) + radiusGrowthRate(tr);
    return conservativeRoot(gap, L, tStart, tEnd);
}

inline ContactPrediction predictWallContact(const Trajectory& tr, const Segment& s,
                                            double tStart, double tEnd) {
    if (tEnd != INF) {
        double reach = (maxSpeedUntil(tr, tEnd) + radiusGrowthRate(tr)) * (tEnd - tStart);
        if (wallGapAt(tr, s, tStart) > reach + CONTACT_EPS) {
            return noContact();
        }
    }
    if (tr.phase == Phase::Recovery) {
        return nextWallContactConservative(tr, s, tStart, tEnd);
    }
    return nextWallContactLinear(tr, s, tStart, tEnd);
}

// ---------------------------------------------------------------------------
// Cruce de una recta horizontal hacia abajo (puerta y = 0, retiro y = -1)
// ---------------------------------------------------------------------------
inline double timeToReachY(const Trajectory& tr, double Y, double tNow) {
    if (tr.dir.y >= 0.0) {
        return INF;                         // no baja
    }
    double dist = (tr.x0.y - Y) / (-tr.dir.y);   // distancia sobre dir desde x0
    if (dist < 0.0) {
        return INF;                         // el tramo empezó por debajo
    }
    double t = INF;
    if (tr.phase == Phase::Free) {
        t = tr.t0 + dist / VD;
    } else if (tr.phase == Phase::Escape) {
        t = tr.t0 + dist / VE;
    } else {
        double sigma = recoveryTimeForDistance(recoveryDistance(tr.t0 - tr.tRec0) + dist);
        t = tr.tRec0 + sigma;
    }
    if (t > tr.phaseEnd() || t < tNow) {
        return INF;
    }
    return t;
}

// ---------------------------------------------------------------------------
// Recinto: cuadrado [0, side]^2, puerta centrada en y = 0
// ---------------------------------------------------------------------------
struct Board {
    double side;
    double doorLeft;
    double doorRight;
    std::vector<Segment> walls;
    Vector2D doorTarget;
    Vector2D exitTarget;
    double removeY;
};

inline Board makeBoard(double side, double doorWidth) {
    Board b;
    b.side = side;
    b.doorLeft = 0.5 * side - 0.5 * doorWidth;
    b.doorRight = 0.5 * side + 0.5 * doorWidth;
    Segment s;
    s.a = Vector2D(0.0, 0.0);         s.b = Vector2D(0.0, side);          b.walls.push_back(s);
    s.a = Vector2D(side, 0.0);        s.b = Vector2D(side, side);         b.walls.push_back(s);
    s.a = Vector2D(0.0, side);        s.b = Vector2D(side, side);         b.walls.push_back(s);
    s.a = Vector2D(0.0, 0.0);         s.b = Vector2D(b.doorLeft, 0.0);    b.walls.push_back(s);
    s.a = Vector2D(b.doorRight, 0.0); s.b = Vector2D(side, 0.0);          b.walls.push_back(s);
    b.doorTarget = Vector2D(0.5 * side, 0.0);
    b.exitTarget = Vector2D(0.5 * side, -10.0);
    b.removeY = -1.0;
    return b;
}

// ---------------------------------------------------------------------------
// Estado y reglas comunes a los dos motores (fuerza bruta y agenda).
// Los dos motores solo difieren en cómo encuentran el próximo grupo de eventos.
// ---------------------------------------------------------------------------
struct Agent {
    int id;
    Trajectory tr;
    Vector2D target;
    bool crossed;
    bool active;
};

enum class EventType { PairContact, WallContact, EscapeEnd, RecoveryEnd, DoorCross, Remove };

struct Event {
    double time;
    EventType type;
    int i;
    int j;      // otro agente, índice de pared, o -1
};

struct ContactSet {
    std::vector<int> touching;
    std::vector<Vector2D> points;
};

class EdCore {
public:
    Board board;
    std::vector<Agent> agents;
    double now;
    long groups;
    long phantomContacts;
    long overlapViolations;
    double worstPenetration;
    std::vector<Event> log;
    std::vector<double> groupTimes;
    bool keepLog;
    bool checkInvariants;

    explicit EdCore(const Board& b)
        : board(b), now(0.0), groups(0), phantomContacts(0),
          overlapViolations(0), worstPenetration(0.0), keepLog(false),
          checkInvariants(true) {}

    void addAgent(int id, const Vector2D& pos, const Vector2D& target, bool crossed) {
        Agent a;
        a.id = id;
        a.target = target;
        a.crossed = crossed;
        a.active = true;
        a.tr.phase = Phase::Free;
        a.tr.t0 = now;
        a.tr.x0 = pos;
        a.tr.dir = normalizedOrZero(target - pos);
        a.tr.tRec0 = 0.0;
        a.tr.version = 0;
        agents.push_back(a);
    }

    int activeCount() const {
        int c = 0;
        for (int i = 0; i < static_cast<int>(agents.size()); ++i) {
            if (agents[i].active) {
                c = c + 1;
            }
        }
        return c;
    }

    double doorCrossTime(int i) const {
        const Agent& a = agents[i];
        if (a.crossed) {
            return INF;
        }
        double t = timeToReachY(a.tr, 0.0, now);
        if (t == INF) {
            return INF;
        }
        double x = a.tr.positionAt(t).x;
        if (x < board.doorLeft || x > board.doorRight) {
            return INF;
        }
        return t;
    }

    double removeTime(int i) const {
        return timeToReachY(agents[i].tr, board.removeY, now);
    }

    // Hasta cuándo valen las fórmulas del tramo actual.
    double validUntil(int i) const {
        double t = agents[i].tr.phaseEnd();
        t = std::min(t, doorCrossTime(i));
        t = std::min(t, removeTime(i));
        return t;
    }

    ContactSet contactSetAt(int i, double t) const {
        ContactSet cs;
        Vector2D xi = agents[i].tr.positionAt(t);
        for (int j = 0; j < static_cast<int>(agents.size()); ++j) {
            if (j == i || !agents[j].active) {
                continue;
            }
            if (gapAt(agents[i].tr, agents[j].tr, t) <= CONTACT_EPS) {
                cs.touching.push_back(j);
                cs.points.push_back(agents[j].tr.positionAt(t));
            }
        }
        for (int w = 0; w < static_cast<int>(board.walls.size()); ++w) {
            if (wallGapAt(agents[i].tr, board.walls[w], t) <= CONTACT_EPS) {
                cs.points.push_back(closestPoint(board.walls[w], xi));
            }
        }
        return cs;
    }

protected:
    // Eventos propios de un agente (no dependen de nadie más).
    void ownEvents(int i, std::vector<Event>& out) const {
        const Trajectory& tr = agents[i].tr;
        if (tr.phase == Phase::Escape) {
            pushEvent(out, tr.phaseEnd(), EventType::EscapeEnd, i, -1);
        }
        if (tr.phase == Phase::Recovery) {
            pushEvent(out, tr.phaseEnd(), EventType::RecoveryEnd, i, -1);
        }
        pushEvent(out, doorCrossTime(i), EventType::DoorCross, i, -1);
        pushEvent(out, removeTime(i), EventType::Remove, i, -1);
    }

    void wallEvents(int i, double horizon, std::vector<Event>& out) const {
        for (int w = 0; w < static_cast<int>(board.walls.size()); ++w) {
            ContactPrediction c = predictWallContact(agents[i].tr, board.walls[w], now, horizon);
            if (c.found) {
                pushEvent(out, c.time, EventType::WallContact, i, w);
            }
        }
    }

    void pairEvent(int i, int j, double horizon, std::vector<Event>& out) const {
        ContactPrediction c = predictContact(agents[i].tr, agents[j].tr, now, horizon);
        if (c.found) {
            pushEvent(out, c.time, EventType::PairContact, i, j);
        }
    }

    static void pushEvent(std::vector<Event>& v, double t, EventType type, int i, int j) {
        if (t == INF) {
            return;
        }
        Event e;
        e.time = t;
        e.type = type;
        e.i = i;
        e.j = j;
        v.push_back(e);
    }

    // Aplica un grupo de eventos simultáneos y avanza el reloj a tg.
    void applyGroup(const std::vector<Event>& group, double tg) {
        processGroup(group, tg);
        now = tg;
        groups = groups + 1;
        if (keepLog) {
            groupTimes.push_back(tg);
            for (int k = 0; k < static_cast<int>(group.size()); ++k) {
                log.push_back(group[k]);
            }
        }
        if (checkInvariants) {
            checkOverlaps(tg);
        }
    }

    void startFree(int i, double t) {
        Trajectory& tr = agents[i].tr;
        Vector2D x = tr.positionAt(t);
        tr.phase = Phase::Free;
        tr.t0 = t;
        tr.x0 = x;
        tr.dir = normalizedOrZero(agents[i].target - x);
        tr.version = tr.version + 1;
    }

    void startRecovery(int i, double t) {
        Trajectory& tr = agents[i].tr;
        Vector2D x = tr.positionAt(t);
        tr.phase = Phase::Recovery;
        tr.t0 = t;
        tr.x0 = x;
        tr.dir = normalizedOrZero(agents[i].target - x);
        tr.tRec0 = t;
        tr.version = tr.version + 1;
    }

    // Recuperación en curso que cambia de dirección (cambio de target).
    void redirectRecovery(int i, double t) {
        Trajectory& tr = agents[i].tr;
        Vector2D x = tr.positionAt(t);
        tr.t0 = t;
        tr.x0 = x;
        tr.dir = normalizedOrZero(agents[i].target - x);
        tr.version = tr.version + 1;
    }

    void startEscape(int i, const std::vector<Vector2D>& points, double t) {
        Trajectory& tr = agents[i].tr;
        Vector2D xi = tr.positionAt(t);
        Vector2D sum(0.0, 0.0);
        for (int k = 0; k < static_cast<int>(points.size()); ++k) {
            sum = sum + normalizedOrZero(xi - points[k]);
        }
        tr.phase = Phase::Escape;
        tr.t0 = t;
        tr.x0 = xi;
        tr.dir = normalizedOrZero(sum);
        tr.version = tr.version + 1;
    }

    void processGroup(const std::vector<Event>& group, double tg) {
        int n = static_cast<int>(agents.size());
        std::vector<bool> contactEvent(n, false);
        std::vector<bool> escapeEnds(n, false);

        // 1) Cambios continuos (no alteran radios ni posiciones en tg).
        for (int k = 0; k < static_cast<int>(group.size()); ++k) {
            const Event& e = group[k];
            if (e.type == EventType::Remove && agents[e.i].active) {
                agents[e.i].active = false;
                agents[e.i].tr.version = agents[e.i].tr.version + 1;
            }
        }
        for (int k = 0; k < static_cast<int>(group.size()); ++k) {
            const Event& e = group[k];
            if (!agents[e.i].active) {
                continue;
            }
            if (e.type == EventType::RecoveryEnd) {
                startFree(e.i, tg);
            }
        }
        for (int k = 0; k < static_cast<int>(group.size()); ++k) {
            const Event& e = group[k];
            if (!agents[e.i].active || e.type != EventType::DoorCross) {
                continue;
            }
            Agent& a = agents[e.i];
            a.crossed = true;
            a.target = board.exitTarget;
            if (a.tr.phase == Phase::Free) {
                startFree(e.i, tg);
            } else if (a.tr.phase == Phase::Recovery) {
                redirectRecovery(e.i, tg);
            } else {
                // Escape: la dirección se mantiene, pero el horizonte de validez
                // cambió (ya no hay cruce de puerta): sus eventos deben recalcularse.
                a.tr.version = a.tr.version + 1;
            }
        }

        // 2) Agentes afectados por contactos o fin de escape.
        std::vector<int> affected;
        std::vector<bool> inAffected(n, false);
        for (int k = 0; k < static_cast<int>(group.size()); ++k) {
            const Event& e = group[k];
            if (e.type == EventType::PairContact) {
                if (agents[e.i].active && agents[e.j].active) {
                    contactEvent[e.i] = true;
                    contactEvent[e.j] = true;
                    addAffected(affected, inAffected, e.i);
                    addAffected(affected, inAffected, e.j);
                }
            } else if (e.type == EventType::WallContact) {
                if (agents[e.i].active) {
                    contactEvent[e.i] = true;
                    addAffected(affected, inAffected, e.i);
                }
            } else if (e.type == EventType::EscapeEnd) {
                if (agents[e.i].active) {
                    escapeEnds[e.i] = true;
                    addAffected(affected, inAffected, e.i);
                }
            }
        }

        // 3) Foto: todos los conjuntos de contacto ANTES de modificar radios.
        //    Cierre simétrico: quien toca a un afectado también se reevalúa.
        std::vector<ContactSet> sets(n);
        for (int k = 0; k < static_cast<int>(affected.size()); ++k) {
            int i = affected[k];
            sets[i] = contactSetAt(i, tg);
            for (int m = 0; m < static_cast<int>(sets[i].touching.size()); ++m) {
                addAffected(affected, inAffected, sets[i].touching[m]);
            }
        }

        // 4) Aplicar.
        for (int k = 0; k < static_cast<int>(affected.size()); ++k) {
            int i = affected[k];
            if (!sets[i].points.empty()) {
                startEscape(i, sets[i].points, tg);
            } else if (escapeEnds[i]) {
                startRecovery(i, tg);
            } else if (contactEvent[i]) {
                phantomContacts = phantomContacts + 1;
                // Su predicción no se cumplió: forzar que se recalculen sus eventos.
                agents[i].tr.version = agents[i].tr.version + 1;
            }
        }
    }

    static void addAffected(std::vector<int>& affected, std::vector<bool>& inAffected, int i) {
        if (!inAffected[i]) {
            inAffected[i] = true;
            affected.push_back(i);
        }
    }

    // Invariante: solo puede haber solapamiento si los involucrados están escapando.
    // Cuesta O(N^2) por grupo: desactivar (checkInvariants = false) para N grandes.
    void checkOverlaps(double t) {
        int n = static_cast<int>(agents.size());
        for (int i = 0; i < n; ++i) {
            if (!agents[i].active) {
                continue;
            }
            for (int j = i + 1; j < n; ++j) {
                if (!agents[j].active) {
                    continue;
                }
                double g = gapAt(agents[i].tr, agents[j].tr, t);
                worstPenetration = std::min(worstPenetration, g);
                bool bothEscaping = agents[i].tr.phase == Phase::Escape
                                 && agents[j].tr.phase == Phase::Escape;
                if (g < -DEPTH_TOL && !bothEscaping) {
                    overlapViolations = overlapViolations + 1;
                }
            }
            for (int w = 0; w < static_cast<int>(board.walls.size()); ++w) {
                double g = wallGapAt(agents[i].tr, board.walls[w], t);
                worstPenetration = std::min(worstPenetration, g);
                if (g < -DEPTH_TOL && agents[i].tr.phase != Phase::Escape) {
                    overlapViolations = overlapViolations + 1;
                }
            }
        }
    }
};

// Primer grupo de eventos simultáneos dentro de una lista de candidatos.
inline double extractGroup(const std::vector<Event>& cand, std::vector<Event>& group) {
    double tMin = INF;
    for (int k = 0; k < static_cast<int>(cand.size()); ++k) {
        tMin = std::min(tMin, cand[k].time);
    }
    double tg = tMin;
    for (int k = 0; k < static_cast<int>(cand.size()); ++k) {
        if (cand[k].time <= tMin + GROUP_TOL) {
            group.push_back(cand[k]);
            tg = std::max(tg, cand[k].time);
        }
    }
    return tg;
}

// ---------------------------------------------------------------------------
// Motor 1: fuerza bruta. Recalcula todos los eventos en cada paso. Es el
// oráculo para verificar el motor con agenda; no escala a N grandes.
// ---------------------------------------------------------------------------
class BruteForceEdSim : public EdCore {
public:
    explicit BruteForceEdSim(const Board& b) : EdCore(b) {}

    bool step() {
        int n = static_cast<int>(agents.size());
        std::vector<double> horizon(n, INF);
        for (int i = 0; i < n; ++i) {
            if (agents[i].active) {
                horizon[i] = validUntil(i);
            }
        }
        std::vector<Event> cand;
        for (int i = 0; i < n; ++i) {
            if (!agents[i].active) {
                continue;
            }
            ownEvents(i, cand);
            wallEvents(i, horizon[i], cand);
            for (int j = i + 1; j < n; ++j) {
                if (agents[j].active) {
                    pairEvent(i, j, std::min(horizon[i], horizon[j]), cand);
                }
            }
        }
        if (cand.empty()) {
            return false;
        }
        std::vector<Event> group;
        double tg = extractGroup(cand, group);
        applyGroup(group, tg);
        return true;
    }
};

// ---------------------------------------------------------------------------
// Motor 2: agenda con heap y versiones.
// Cada evento guarda la versión de sus agentes al momento de predecirse.
// Al sacarlo del heap, si alguna versión cambió, el evento es obsoleto.
// Después de cada grupo solo se reprograman los agentes cuya trayectoria cambió.
// ---------------------------------------------------------------------------
struct QueuedEvent {
    Event e;
    int vi;
    int vj;
};

struct LaterFirst {
    bool operator()(const QueuedEvent& a, const QueuedEvent& b) const {
        return a.e.time > b.e.time;
    }
};

class EventDrivenSim : public EdCore {
public:
    long pushed;
    long popped;
    long stale;

    explicit EventDrivenSim(const Board& b)
        : EdCore(b), pushed(0), popped(0), stale(0), initialized(false) {}

    bool step() {
        if (!initialized) {
            scheduleAll();
            initialized = true;
        }

        // Sacar el primer grupo de eventos vigentes.
        std::vector<Event> group;
        double tMin = INF;
        double tg = INF;
        while (!queue.empty()) {
            const QueuedEvent& top = queue.top();
            if (!isCurrent(top)) {
                queue.pop();
                stale = stale + 1;
                continue;
            }
            if (group.empty()) {
                tMin = top.e.time;
                tg = tMin;
            } else if (top.e.time > tMin + GROUP_TOL) {
                break;
            }
            tg = std::max(tg, top.e.time);
            group.push_back(top.e);
            queue.pop();
            popped = popped + 1;
        }
        if (group.empty()) {
            return false;
        }

        int n = static_cast<int>(agents.size());
        std::vector<int> before(n);
        for (int i = 0; i < n; ++i) {
            before[i] = agents[i].tr.version;
        }

        applyGroup(group, tg);

        // Reprogramar solo a los agentes activos cuya trayectoria cambió.
        std::vector<bool> dirty(n, false);
        std::vector<int> dirtyList;
        for (int i = 0; i < n; ++i) {
            if (agents[i].active && agents[i].tr.version != before[i]) {
                dirty[i] = true;
                dirtyList.push_back(i);
            }
        }
        std::vector<double> horizon = horizons();
        for (int k = 0; k < static_cast<int>(dirtyList.size()); ++k) {
            scheduleFor(dirtyList[k], dirty, horizon);
        }
        return true;
    }

private:
    std::priority_queue<QueuedEvent, std::vector<QueuedEvent>, LaterFirst> queue;
    bool initialized;

    std::vector<double> horizons() const {
        int n = static_cast<int>(agents.size());
        std::vector<double> horizon(n, INF);
        for (int i = 0; i < n; ++i) {
            if (agents[i].active) {
                horizon[i] = validUntil(i);
            }
        }
        return horizon;
    }

    bool isCurrent(const QueuedEvent& q) const {
        const Agent& a = agents[q.e.i];
        if (!a.active || a.tr.version != q.vi) {
            return false;
        }
        if (q.e.type == EventType::PairContact) {
            const Agent& b = agents[q.e.j];
            if (!b.active || b.tr.version != q.vj) {
                return false;
            }
        }
        return true;
    }

    void enqueue(const std::vector<Event>& events) {
        for (int k = 0; k < static_cast<int>(events.size()); ++k) {
            QueuedEvent q;
            q.e = events[k];
            q.vi = agents[q.e.i].tr.version;
            q.vj = -1;
            if (q.e.type == EventType::PairContact) {
                q.vj = agents[q.e.j].tr.version;
            }
            queue.push(q);
            pushed = pushed + 1;
        }
    }

    void scheduleAll() {
        int n = static_cast<int>(agents.size());
        std::vector<double> horizon = horizons();
        std::vector<Event> events;
        for (int i = 0; i < n; ++i) {
            if (!agents[i].active) {
                continue;
            }
            ownEvents(i, events);
            wallEvents(i, horizon[i], events);
            for (int j = i + 1; j < n; ++j) {
                if (agents[j].active) {
                    pairEvent(i, j, std::min(horizon[i], horizon[j]), events);
                }
            }
        }
        enqueue(events);
    }

    // Eventos de un agente cuya trayectoria cambió. Si el otro agente del par
    // también cambió, el par se programa una sola vez (desde el de índice menor).
    void scheduleFor(int i, const std::vector<bool>& dirty, const std::vector<double>& horizon) {
        std::vector<Event> events;
        ownEvents(i, events);
        wallEvents(i, horizon[i], events);
        int n = static_cast<int>(agents.size());
        for (int j = 0; j < n; ++j) {
            if (j == i || !agents[j].active) {
                continue;
            }
            if (dirty[j] && j < i) {
                continue;
            }
            pairEvent(i, j, std::min(horizon[i], horizon[j]), events);
        }
        enqueue(events);
    }
};
