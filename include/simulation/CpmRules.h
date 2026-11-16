#pragma once

#include "simulation/CpmParameters.h"

#include <algorithm>
#include <cmath>

// Reglas CPM de un paso, compartidas por el TSDMD y por la parte TS del híbrido
// para que los dos hagan exactamente la misma aritmética.
namespace simulation::cpm {

// Radio después de un paso sin contacto.
inline double recoveredRadius(double radius) {
    return std::min(maxRadius, radius + maxRadius * dt / tau);
}

// Rapidez deseada para un radio dado.
inline double speedForRadius(double radius) {
    return desiredSpeed * std::pow((radius - minRadius) / (maxRadius - minRadius), beta);
}

}
