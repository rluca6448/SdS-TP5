#pragma once

// Parámetros CPM compartidos por los dos motores (TSDMD y EDMD).
// Baglietto y Parisi (2011), secciones II y III.
namespace simulation::cpm {

constexpr double maxRadius = 0.25;                       // [m]
constexpr double minRadius = 0.10;                       // [m]
constexpr double desiredSpeed = 1.5;                     // [m/s]
constexpr double escapeSpeed = desiredSpeed;             // [m/s]
constexpr double tau = 0.5;                              // [s]
constexpr double beta = 0.9;
constexpr double dt = minRadius / (2 * escapeSpeed);     // [s] = 1/30

}
