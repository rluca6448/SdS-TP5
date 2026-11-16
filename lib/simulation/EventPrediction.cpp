#include "simulation/EventPrediction.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace simulation::edmd {

namespace {

// Distancia recorrida en recuperación, sigma = tiempo desde que el radio valía rmin.
// Integral exacta de v = vd (sigma / T)^beta.
double recoveryDistance(double sigma) {
    if (sigma <= 0.0) {
        return 0.0;
    }
    const double s = std::min(sigma, recoveryDuration) / recoveryDuration;
    return cpm::desiredSpeed * recoveryDuration / (1.0 + cpm::beta) * std::pow(s, 1.0 + cpm::beta);
}

// Inversa de recoveryDistance: sigma para recorrer dist, o infinity si no alcanza.
double recoveryTimeForDistance(double dist) {
    const double maxDistance = cpm::desiredSpeed * recoveryDuration / (1.0 + cpm::beta);
    if (dist > maxDistance) {
        return infinity;
    }
    if (dist <= 0.0) {
        return 0.0;
    }
    return recoveryDuration * std::pow(dist / maxDistance, 1.0 / (1.0 + cpm::beta));
}

// Velocidad de un tramo rectilíneo (Free o Escape).
types::Vector2D linearVelocity(const Trajectory& a) {
    if (a.phase == Phase::Free) {
        return a.direction * cpm::desiredSpeed;
    }
    return a.direction * cpm::escapeSpeed;
}

// Rapidez máxima del tramo hasta tEnd (en recuperación crece con el tiempo).
double maxSpeedUntil(const Trajectory& a, double tEnd) {
    if (a.direction.norm() == 0.0) {
        return 0.0;
    }
    if (a.phase == Phase::Free) {
        return cpm::desiredSpeed;
    }
    if (a.phase == Phase::Escape) {
        return cpm::escapeSpeed;
    }
    const double s = std::clamp((tEnd - a.recoveryStart) / recoveryDuration, 0.0, 1.0);
    return cpm::desiredSpeed * std::pow(s, cpm::beta);
}

double radiusGrowthRate(const Trajectory& a) {
    return a.phase == Phase::Recovery ? growthRate : 0.0;
}

// Cota de qué tan rápido puede achicarse el hueco entre a y b hasta tEnd.
double pairClosingBound(const Trajectory& a, const Trajectory& b, double tEnd) {
    return maxSpeedUntil(a, tEnd) + maxSpeedUntil(b, tEnd)
         + radiusGrowthRate(a) + radiusGrowthRate(b);
}

// El instante de contacto es donde el hueco vale contactTolerance / 2, de modo
// que al procesar el evento el contacto se detecta con margen.
constexpr double contactTarget = 0.5 * contactTolerance;

// Avance conservador: nunca salta más de lo que el hueco puede cerrarse.
// Sirve para cualquier tramo, incluida la recuperación (sin fórmula cerrada).
ContactPrediction conservativeRoot(const std::function<double(double)>& gap,
                                   double closingBound, double tStart, double tEnd) {
    ContactPrediction none;
    double previousGap = gap(tStart);
    if (previousGap <= contactTolerance || closingBound <= 0.0 || tEnd <= tStart) {
        return none;
    }
    const double minStep = 2.0 * depthTolerance / closingBound;
    const int maxSteps = 1000000;

    double previous = tStart;
    for (int steps = 0; previous < tEnd; ++steps) {
        if (steps > maxSteps) {
            throw std::runtime_error("EDMD: conservative advancement did not converge.");
        }
        const double step = std::max((previousGap - contactTarget) / closingBound, minStep);
        const double next = std::min(previous + step, tEnd);
        const double nextGap = gap(next);
        if (nextGap <= contactTarget) {
            double low = previous;
            double high = next;
            for (int i = 0; i < 100 && high - low > timeTolerance; ++i) {
                const double middle = 0.5 * (low + high);
                if (gap(middle) <= contactTarget) {
                    high = middle;
                } else {
                    low = middle;
                }
            }
            ContactPrediction hit;
            hit.found = true;
            hit.time = high;
            return hit;
        }
        previous = next;
        previousGap = nextGap;
    }
    return none;
}

// Primera entrada de p + u h en el disco de radio R centrado en el origen.
// Raíz menor en forma estable: c / q.
double firstEntryIntoDisc(types::Vector2D p, types::Vector2D u, double radius) {
    const double qa = u.dot(u);
    const double qb = 2.0 * p.dot(u);
    const double qc = p.dot(p) - radius * radius;
    if (qc <= 0.0 || qb >= 0.0) {
        return infinity;               // ya adentro, o no se acerca
    }
    const double discriminant = qb * qb - 4.0 * qa * qc;
    if (discriminant < 0.0) {
        return infinity;               // pasa sin entrar
    }
    const double q = 0.5 * (-qb + std::sqrt(discriminant));
    return qc / q;
}

ContactPrediction pairContactLinear(const Trajectory& a, const Trajectory& b,
                                    double tStart, double tEnd) {
    ContactPrediction none;
    if (pairGap(a, b, tStart) <= contactTolerance) {
        return none;
    }
    const types::Vector2D p = a.positionAt(tStart) - b.positionAt(tStart);
    const types::Vector2D u = linearVelocity(a) - linearVelocity(b);
    const double radius = a.radiusAt(tStart) + b.radiusAt(tStart) + contactTarget;
    const double h = firstEntryIntoDisc(p, u, radius);
    if (h == infinity || tStart + h > tEnd) {
        return none;
    }
    ContactPrediction hit;
    hit.found = true;
    hit.time = tStart + h;
    return hit;
}

// Pared con radio fijo: la zona de contacto es una cápsula (banda + extremos).
ContactPrediction wallContactLinear(const Trajectory& a, const types::Wall& wall,
                                    double tStart, double tEnd) {
    ContactPrediction none;
    if (wallGap(a, wall, tStart) <= contactTolerance) {
        return none;
    }
    const types::Vector2D p = a.positionAt(tStart);
    const types::Vector2D v = linearVelocity(a);
    const double radius = a.radiusAt(tStart) + contactTarget;

    const types::Vector2D along = wall.end - wall.start;
    const double length = along.norm();
    const types::Vector2D tangent = along * (1.0 / length);
    const types::Vector2D normal{-tangent.y, tangent.x};

    double best = infinity;
    const double normalDistance = normal.dot(p - wall.start);
    const double normalSpeed = normal.dot(v);
    if (std::abs(normalDistance) > radius && normalSpeed != 0.0) {
        const double side = normalDistance > 0.0 ? 1.0 : -1.0;
        const double h = (side * radius - normalDistance) / normalSpeed;
        if (h >= 0.0) {
            const double projection = tangent.dot(p + v * h - wall.start);
            if (projection >= 0.0 && projection <= length) {
                best = h;
            }
        }
    }
    best = std::min(best, firstEntryIntoDisc(p - wall.start, v, radius));
    best = std::min(best, firstEntryIntoDisc(p - wall.end, v, radius));

    if (best == infinity || tStart + best > tEnd) {
        return none;
    }
    ContactPrediction hit;
    hit.found = true;
    hit.time = tStart + best;
    return hit;
}

}

double Trajectory::radiusAt(double t) const {
    if (phase == Phase::Free) {
        return cpm::maxRadius;
    }
    if (phase == Phase::Escape) {
        return cpm::minRadius;
    }
    return std::min(cpm::maxRadius, cpm::minRadius + growthRate * (t - recoveryStart));
}

types::Vector2D Trajectory::positionAt(double t) const {
    if (phase == Phase::Free) {
        return x0 + direction * (cpm::desiredSpeed * (t - t0));
    }
    if (phase == Phase::Escape) {
        return x0 + direction * (cpm::escapeSpeed * (t - t0));
    }
    const double distance = recoveryDistance(t - recoveryStart) - recoveryDistance(t0 - recoveryStart);
    return x0 + direction * distance;
}

types::Vector2D Trajectory::velocityAt(double t) const {
    if (phase != Phase::Recovery) {
        return linearVelocity(*this);
    }
    const double s = std::clamp((t - recoveryStart) / recoveryDuration, 0.0, 1.0);
    return direction * (cpm::desiredSpeed * std::pow(s, cpm::beta));
}

double Trajectory::phaseEnd() const {
    if (phase == Phase::Escape) {
        return t0 + escapeDuration;
    }
    if (phase == Phase::Recovery) {
        return recoveryStart + recoveryDuration;
    }
    return infinity;
}

double pairGap(const Trajectory& a, const Trajectory& b, double t) {
    return (a.positionAt(t) - b.positionAt(t)).norm() - (a.radiusAt(t) + b.radiusAt(t));
}

double wallGap(const Trajectory& a, const types::Wall& wall, double t) {
    const types::Vector2D position = a.positionAt(t);
    return (position - wall.closestPoint(position)).norm() - a.radiusAt(t);
}

ContactPrediction predictPairContact(const Trajectory& a, const Trajectory& b,
                                     double tStart, double tEnd) {
    // Filtro de alcance exacto: si el hueco no puede cerrarse antes de tEnd, no hay choque.
    if (tEnd != infinity) {
        const double reach = pairClosingBound(a, b, tEnd) * (tEnd - tStart);
        if (pairGap(a, b, tStart) > reach + contactTolerance) {
            return ContactPrediction{};
        }
    }
    if (a.phase == Phase::Recovery || b.phase == Phase::Recovery) {
        const std::function<double(double)> gap = [&a, &b](double t) { return pairGap(a, b, t); };
        return conservativeRoot(gap, pairClosingBound(a, b, tEnd), tStart, tEnd);
    }
    return pairContactLinear(a, b, tStart, tEnd);
}

ContactPrediction predictWallContact(const Trajectory& a, const types::Wall& wall,
                                     double tStart, double tEnd) {
    if (tEnd != infinity) {
        const double reach = (maxSpeedUntil(a, tEnd) + radiusGrowthRate(a)) * (tEnd - tStart);
        if (wallGap(a, wall, tStart) > reach + contactTolerance) {
            return ContactPrediction{};
        }
    }
    if (a.phase == Phase::Recovery) {
        const std::function<double(double)> gap = [&a, &wall](double t) { return wallGap(a, wall, t); };
        const double closingBound = maxSpeedUntil(a, tEnd) + radiusGrowthRate(a);
        return conservativeRoot(gap, closingBound, tStart, tEnd);
    }
    return wallContactLinear(a, wall, tStart, tEnd);
}

double timeToReachY(const Trajectory& a, double y, double now) {
    if (a.direction.y >= 0.0) {
        return infinity;
    }
    const double distance = (a.x0.y - y) / (-a.direction.y);   // sobre direction, desde x0
    if (distance < 0.0) {
        return infinity;
    }
    double t = infinity;
    if (a.phase == Phase::Free) {
        t = a.t0 + distance / cpm::desiredSpeed;
    } else if (a.phase == Phase::Escape) {
        t = a.t0 + distance / cpm::escapeSpeed;
    } else {
        const double covered = recoveryDistance(a.t0 - a.recoveryStart);
        t = a.recoveryStart + recoveryTimeForDistance(covered + distance);
    }
    if (t > a.phaseEnd() || t < now) {
        return infinity;
    }
    return t;
}

}
