#pragma once

#include "simulation/CpmParameters.h"
#include "types/Board.h"
#include "types/Vector2D.h"

#include <limits>

// Trayectorias CPM en tiempo continuo y predicción de eventos para el EDMD.
namespace simulation::edmd {

constexpr double growthRate = cpm::maxRadius / cpm::tau;                          // [m/s]
constexpr double recoveryDuration = (cpm::maxRadius - cpm::minRadius) / growthRate; // [s] 0,3
constexpr double escapeDuration = cpm::dt;                                         // [s] 1/30
constexpr double infinity = std::numeric_limits<double>::infinity();

constexpr double contactTolerance = 1e-9;   // [m] hueco <= esto es contacto
constexpr double depthTolerance = 1e-6;     // [m] penetración máxima que puede pasar inadvertida
constexpr double timeTolerance = 1e-12;     // [s] precisión de la bisección
constexpr double groupTolerance = 1e-11;    // [s] eventos más cercanos que esto son simultáneos
// Detector de cascadas: una corrida sana procesa del orden de 10 grupos por ms.
constexpr double progressInterval = 1e-3;          // [s] avance mínimo esperado...
constexpr long maxGroupsWithoutProgress = 100000;  // ...cada esta cantidad de grupos

enum class Phase { Free, Escape, Recovery };

// Tramo de movimiento con fórmula cerrada, válido desde t0 hasta su fin de fase
// (o hasta que un evento lo reemplace).
struct Trajectory {
    Phase phase = Phase::Free;
    double t0 = 0.0;                 // inicio del tramo
    types::Vector2D x0;              // posición en t0
    types::Vector2D direction;       // unitaria, o cero
    double recoveryStart = 0.0;      // solo Recovery: instante en que el radio valía rmin
    int version = 0;                 // cambia cada vez que cambia el tramo

    double radiusAt(double t) const;
    types::Vector2D positionAt(double t) const;
    types::Vector2D velocityAt(double t) const;
    double phaseEnd() const;
};

struct ContactPrediction {
    bool found = false;
    double time = 0.0;
};

// Hueco entre discos (o entre disco y pared): > 0 separados, <= 0 en contacto.
double pairGap(const Trajectory& a, const Trajectory& b, double t);
double wallGap(const Trajectory& a, const types::Wall& wall, double t);

// Primera entrada en contacto en [tStart, tEnd]. Un contacto ya existente en
// tStart no es un evento futuro y no se informa.
ContactPrediction predictPairContact(const Trajectory& a, const Trajectory& b,
                                     double tStart, double tEnd);
ContactPrediction predictWallContact(const Trajectory& a, const types::Wall& wall,
                                     double tStart, double tEnd);

// Instante en que el tramo cruza la recta horizontal y hacia abajo, o infinity.
double timeToReachY(const Trajectory& a, double y, double now);

}
