#include "simulation/SimulationHybrid.h"
#include "simulation/CpmRules.h"

#include <cmath>
#include <random>

namespace simulation {

SimulationHybrid::SimulationHybrid(int particleCount, unsigned seed, types::Board board,
                                   int anticipation, bool forceTimeDriven)
    : board(board), anticipation(anticipation), forceTimeDriven(forceTimeDriven),
      contactGrid(board.getSide(), 2 * cpm::maxRadius), boundaryGrid(board.getSide(), safeDistance) {
    initializeParticles(particleCount, seed);
    flights.resize(particles.size());
    indexOf.resize(particles.size());
    version.assign(particles.size(), 0);
    for (std::size_t i = 0; i < particles.size(); ++i) {
        indexOf[i] = static_cast<int>(i);
    }
}

// Igual que Simulation::initializeParticles: misma semilla, mismo estado inicial.
// Todos empiezan en TS; en el primer paso, los que están libres pasan a ED.
void SimulationHybrid::initializeParticles(int particleCount, unsigned seed) {
    std::mt19937 random(seed);
    std::uniform_real_distribution<double> coordinate(cpm::maxRadius, board.getSide() - cpm::maxRadius);
    particles.reserve(static_cast<std::size_t>(particleCount));

    while (particles.size() < static_cast<std::size_t>(particleCount)) {
        const types::Vector2D position{coordinate(random), coordinate(random)};
        bool overlaps = false;
        for (const types::Particle& particle : particles) {
            if ((position - particle.getPosition()).norm() <= cpm::maxRadius + particle.getRadius()) {
                overlaps = true;
                break;
            }
        }
        if (overlaps) {
            continue;
        }
        types::Particle particle(static_cast<int>(particles.size()), position, cpm::maxRadius);
        particle.setTarget(board.getDoorTarget());
        const types::Vector2D direction = particle.getTarget() - position;
        particle.setVelocity(direction.normalized() * cpm::desiredSpeed);
        particles.push_back(particle);
    }
}

// ---------------------------------------------------------------------------
// Paso
// ---------------------------------------------------------------------------

void SimulationHybrid::step() {
    const std::size_t n = particles.size();
    contactGrid.rebuild(particles);

    // 1) Quién es TS en este paso.
    std::vector<char> timeDriven(n, 0);
    if (forceTimeDriven) {
        std::fill(timeDriven.begin(), timeDriven.end(), 1);
    } else {
        classify(timeDriven);
    }
    if (checkMissedContacts) {
        countMissedContacts(timeDriven);
    }

    // 2) Reglas CPM para los TS, idénticas al TSDMD. Los contactos se buscan solo
    //    entre agentes TS: el cierre de la bolsa garantiza que ningún ED toca a nadie.
    std::vector<bool> contacts(n, false);
    std::vector<types::Vector2D> directions(n);
    findContacts(timeDriven, contacts, directions);
    for (std::size_t i = 0; i < n; ++i) {
        types::Particle& particle = particles[i];
        if (!timeDriven[i]) {
            particle.setVelocity(flights[i].trajectory.direction * cpm::desiredSpeed);
            particle.setScheme(types::MotionScheme::EventDriven);
            continue;
        }
        if (contacts[i]) {
            particle.setRadius(cpm::minRadius);
            particle.setVelocity(directions[i].normalized() * cpm::escapeSpeed);
        } else {
            const double radius = cpm::recoveredRadius(particle.getRadius());
            particle.setRadius(radius);
            const double speed = cpm::speedForRadius(radius);
            const types::Vector2D direction = particle.getTarget() - particle.getPosition();
            particle.setVelocity(direction.normalized() * speed);
        }
        particle.setScheme(types::MotionScheme::TimeDriven);
    }

    // 3) Mover: TS con su velocidad, ED con su vuelo. La puerta se comprueba igual
    //    para los dos, sobre el segmento recorrido, como en el TSDMD.
    std::vector<int> redirected;
    for (std::size_t i = 0; i < n; ++i) {
        types::Particle& particle = particles[i];
        const types::Vector2D previous = particle.getPosition();
        const types::Vector2D position = timeDriven[i]
            ? previous + particle.getVelocity() * cpm::dt
            : flights[i].trajectory.positionAt(time + cpm::dt);
        particle.setPosition(position);
        if (previous.y > 0 && position.y <= 0) {
            const double fraction = previous.y / (previous.y - position.y);
            const double crossingX = previous.x + fraction * (position.x - previous.x);
            if (std::abs(crossingX - board.getSide() / 2) <= board.getDoorWidth() / 2) {
                particle.setTarget(board.getExitTarget());
                evacuationTime = time + cpm::dt;
                if (!timeDriven[i]) {
                    redirected.push_back(particle.getId());
                }
            }
        }
    }

    // 4) Retiro, con la misma compactación estable que el TSDMD.
    std::size_t kept = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const int id = particles[i].getId();
        if (particles[i].getPosition().y <= board.getExitTarget().y) {
            indexOf[static_cast<std::size_t>(id)] = -1;
            ++version[static_cast<std::size_t>(id)];
            continue;
        }
        if (kept != i) {
            particles[kept] = particles[i];
            flights[kept] = flights[i];
        }
        indexOf[static_cast<std::size_t>(id)] = static_cast<int>(kept);
        ++kept;
    }
    particles.erase(particles.begin() + static_cast<std::ptrdiff_t>(kept), particles.end());
    flights.erase(flights.begin() + static_cast<std::ptrdiff_t>(kept), flights.end());
    time += cpm::dt;

    // 5) Los ED que cruzaron la puerta siguen volando hacia el target exterior.
    //    Primero se actualizan todos los vuelos y después se predice: si dos cruzan
    //    en el mismo tick, cada uno se predice contra el vuelo nuevo del otro.
    std::vector<std::size_t> redirectedIndices;
    for (int id : redirected) {
        const int index = indexOf[static_cast<std::size_t>(id)];
        if (index >= 0) {
            const std::size_t i = static_cast<std::size_t>(index);
            flights[i] = flightFrom(i, time);
            ++version[static_cast<std::size_t>(id)];
            redirectedIndices.push_back(i);
        }
    }
    std::vector<char> flying(particles.size(), 0);
    for (std::size_t i = 0; i < particles.size(); ++i) {
        flying[i] = isEventDriven(i) ? 1 : 0;
    }
    const std::vector<char> noneLeaving(particles.size(), 0);
    std::vector<Prediction> predicted;
    for (std::size_t i : redirectedIndices) {
        predictFlight(i, flights[i], time, flying, noneLeaving, flights, predicted);
    }
    enqueue(predicted);
}

// ---------------------------------------------------------------------------
// Clasificación de la bolsa, reconstruida en cada paso desde el estado actual
// ---------------------------------------------------------------------------

void SimulationHybrid::classify(std::vector<char>& timeDriven) {
    const std::size_t n = particles.size();
    boundaryGrid.rebuild(particles);
    std::vector<std::size_t> members;
    std::vector<char> leaving(n, 0);
    std::vector<char> flying(n, 0);   // agentes que, por ahora, van a volar en ED
    long unused = 0;

    // a) Obligatorios: los TS del paso anterior que no están recuperados o están
    //    en contacto. Los demás TS podrían salir.
    for (std::size_t i = 0; i < n; ++i) {
        if (isEventDriven(i)) {
            flying[i] = 1;
        } else if (particles[i].getRadius() < cpm::maxRadius || touchesSomething(i)) {
            markTimeDriven(i, timeDriven, members, flying, unused);
        } else {
            leaving[i] = 1;
            flying[i] = 1;
        }
    }

    // b) Agentes ED con un choque previsto dentro de k * dt.
    const double horizonEnd = time + anticipation * cpm::dt + 1e-9;
    while (!predictions.empty() && predictions.top().time <= horizonEnd) {
        const Prediction prediction = predictions.top();
        predictions.pop();
        if (!isCurrent(prediction)) {
            continue;
        }
        markTimeDriven(static_cast<std::size_t>(indexOf[static_cast<std::size_t>(prediction.idA)]),
                       timeDriven, members, flying, predictedEntries);
        if (!prediction.wall) {
            markTimeDriven(static_cast<std::size_t>(indexOf[static_cast<std::size_t>(prediction.idB)]),
                           timeDriven, members, flying, predictedEntries);
        }
    }

    // c) Cierre: quien puede tocar a un TS en el próximo paso entra a TS.
    includeNeighbours(timeDriven, members, 0, flying);

    // d) Los que todavía pueden salir predicen su vuelo. Si chocarían dentro de
    //    k * dt, se quedan (y su compañero entra). Si no, sus predicciones se agendan.
    std::vector<Flight> tentative(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (leaving[i] && flying[i]) {
            tentative[i] = flightFrom(i, time);
        }
    }
    const std::size_t firstLate = members.size();
    std::vector<Prediction> pending;
    for (std::size_t i = 0; i < n; ++i) {
        if (!leaving[i] || !flying[i]) {
            continue;
        }
        std::vector<Prediction> own;
        predictFlight(i, tentative[i], time, flying, leaving, tentative, own);
        for (const Prediction& prediction : own) {
            if (prediction.time <= horizonEnd) {
                markTimeDriven(i, timeDriven, members, flying, predictedEntries);
                if (!prediction.wall) {
                    markTimeDriven(static_cast<std::size_t>(indexOf[static_cast<std::size_t>(prediction.idB)]),
                                   timeDriven, members, flying, predictedEntries);
                }
            }
        }
        pending.insert(pending.end(), own.begin(), own.end());
    }
    includeNeighbours(timeDriven, members, firstLate, flying);

    // e) Cerrar la clasificación: invalidar los vuelos de los ED que entran a TS,
    //    dar vuelo a los que salen y agendar solo las predicciones entre agentes
    //    que efectivamente quedaron en ED.
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t id = static_cast<std::size_t>(particles[i].getId());
        if (timeDriven[i] && isEventDriven(i)) {
            ++version[id];
        } else if (!timeDriven[i] && leaving[i]) {
            flights[i] = tentative[i];
            ++version[id];
        }
    }
    std::vector<Prediction> kept;
    for (const Prediction& prediction : pending) {
        const std::size_t a = static_cast<std::size_t>(indexOf[static_cast<std::size_t>(prediction.idA)]);
        if (timeDriven[a]) {
            continue;
        }
        if (!prediction.wall &&
            timeDriven[static_cast<std::size_t>(indexOf[static_cast<std::size_t>(prediction.idB)])]) {
            continue;
        }
        kept.push_back(prediction);
    }
    enqueue(kept);
}

void SimulationHybrid::markTimeDriven(std::size_t i, std::vector<char>& timeDriven,
                                      std::vector<std::size_t>& members, std::vector<char>& flying,
                                      long& counter) {
    if (timeDriven[i]) {
        return;
    }
    timeDriven[i] = 1;
    flying[i] = 0;
    members.push_back(i);
    ++counter;
}

// Recorre la bolsa desde members[start] e incorpora a quien esté a safeDistance o
// menos de algún miembro, hasta que no entre nadie más.
void SimulationHybrid::includeNeighbours(std::vector<char>& timeDriven, std::vector<std::size_t>& members,
                                         std::size_t start, std::vector<char>& flying) {
    for (std::size_t k = start; k < members.size(); ++k) {
        const types::Vector2D position = particles[members[k]].getPosition();
        for (std::size_t c : boundaryGrid.getCandidates(position)) {
            if (!timeDriven[c] && (particles[c].getPosition() - position).norm() <= safeDistance) {
                markTimeDriven(c, timeDriven, members, flying, boundaryEntries);
            }
        }
    }
}

bool SimulationHybrid::touchesSomething(std::size_t i) const {
    const types::Particle& particle = particles[i];
    for (std::size_t j : contactGrid.getCandidates(particle.getPosition())) {
        if (j != i && (particle.getPosition() - particles[j].getPosition()).norm()
                          <= particle.getRadius() + particles[j].getRadius()) {
            return true;
        }
    }
    for (const types::Wall& wall : board.getWalls()) {
        if ((particle.getPosition() - wall.closestPoint(particle.getPosition())).norm() <= particle.getRadius()) {
            return true;
        }
    }
    return false;
}

// Igual que Simulation::findContacts, restringido a pares de agentes TS. Con todos
// en TS recorre lo mismo en el mismo orden.
void SimulationHybrid::findContacts(const std::vector<char>& timeDriven, std::vector<bool>& contacts,
                                    std::vector<types::Vector2D>& directions) const {
    for (std::size_t i = 0; i < particles.size(); ++i) {
        if (!timeDriven[i]) {
            continue;
        }
        const types::Particle& particle = particles[i];
        for (std::size_t j : contactGrid.getCandidates(particle.getPosition())) {
            if (j <= i || !timeDriven[j]) {
                continue;
            }
            const types::Vector2D delta = particle.getPosition() - particles[j].getPosition();
            if (delta.norm() <= particle.getRadius() + particles[j].getRadius()) {
                contacts[i] = contacts[j] = true;
                const types::Vector2D direction = delta.normalized();
                directions[i] += direction;
                directions[j] += direction * -1;
            }
        }
        for (const types::Wall& wall : board.getWalls()) {
            const types::Vector2D delta = particle.getPosition() - wall.closestPoint(particle.getPosition());
            if (delta.norm() <= particle.getRadius()) {
                contacts[i] = true;
                directions[i] += delta.normalized();
            }
        }
    }
}

// Verificación: un contacto con algún agente ED significa que una predicción o el
// cierre de la bolsa fallaron.
void SimulationHybrid::countMissedContacts(const std::vector<char>& timeDriven) {
    for (std::size_t i = 0; i < particles.size(); ++i) {
        const types::Particle& particle = particles[i];
        for (std::size_t j : contactGrid.getCandidates(particle.getPosition())) {
            if (j <= i || (timeDriven[i] && timeDriven[j])) {
                continue;
            }
            if ((particle.getPosition() - particles[j].getPosition()).norm()
                <= particle.getRadius() + particles[j].getRadius()) {
                ++missedContacts;
            }
        }
        if (!timeDriven[i]) {
            for (const types::Wall& wall : board.getWalls()) {
                if ((particle.getPosition() - wall.closestPoint(particle.getPosition())).norm()
                    <= particle.getRadius()) {
                    ++missedContacts;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Vuelos y predicciones
// ---------------------------------------------------------------------------

// Vuelo libre desde la posición actual hacia el target, a vd. Vale hasta el fin
// del tick en que cruza la puerta o se retira: ahí el TSDMD cambia el target.
SimulationHybrid::Flight SimulationHybrid::flightFrom(std::size_t i, double t) const {
    const types::Particle& particle = particles[i];
    Flight flight;
    flight.trajectory.phase = edmd::Phase::Free;
    flight.trajectory.t0 = t;
    flight.trajectory.x0 = particle.getPosition();
    flight.trajectory.direction = (particle.getTarget() - particle.getPosition()).normalized();

    const bool crossed = particle.getTarget().y < 0;
    if (!crossed) {
        const double crossing = edmd::timeToReachY(flight.trajectory, 0.0, t);
        if (crossing != edmd::infinity) {
            flight.limit = tickAtOrAfter(crossing);
        }
    }
    const double removal = edmd::timeToReachY(flight.trajectory, board.getExitTarget().y, t);
    if (removal != edmd::infinity) {
        flight.limit = std::min(flight.limit, tickAtOrAfter(removal));
    }
    return flight;
}

// Primer tick en o después de t (con un margen para no quedarse corto).
double SimulationHybrid::tickAtOrAfter(double t) const {
    return std::ceil(t / cpm::dt - 1e-9) * cpm::dt + 1e-9;
}

bool SimulationHybrid::isCurrent(const Prediction& prediction) const {
    const int a = indexOf[static_cast<std::size_t>(prediction.idA)];
    if (a < 0 || !isEventDriven(static_cast<std::size_t>(a)) ||
        version[static_cast<std::size_t>(prediction.idA)] != prediction.versionA) {
        return false;
    }
    if (prediction.wall) {
        return true;
    }
    const int b = indexOf[static_cast<std::size_t>(prediction.idB)];
    return b >= 0 && isEventDriven(static_cast<std::size_t>(b)) &&
           version[static_cast<std::size_t>(prediction.idB)] == prediction.versionB;
}

// Choques futuros del vuelo de i contra las paredes y contra los demás que vuelan.
// Los pares entre dos que salen de TS se calculan una sola vez (desde el menor).
void SimulationHybrid::predictFlight(std::size_t i, const Flight& flight, double t,
                                     const std::vector<char>& flying, const std::vector<char>& leaving,
                                     const std::vector<Flight>& tentative, std::vector<Prediction>& out) {
    const int id = particles[i].getId();
    const std::array<types::Wall, 5>& walls = board.getWalls();
    for (std::size_t w = 0; w < walls.size(); ++w) {
        const edmd::ContactPrediction contact = edmd::predictWallContact(flight.trajectory, walls[w], t, flight.limit);
        if (contact.found) {
            Prediction prediction;
            prediction.time = contact.time;
            prediction.idA = id;
            prediction.idB = static_cast<int>(w);
            prediction.wall = true;
            out.push_back(prediction);
        }
    }
    for (std::size_t j = 0; j < particles.size(); ++j) {
        if (j == i || !flying[j] || (leaving[j] && leaving[i] && j < i)) {
            continue;
        }
        const Flight& other = leaving[j] ? tentative[j] : flights[j];
        const double limit = std::min(flight.limit, other.limit);
        ++pairPredictions;
        const edmd::ContactPrediction contact =
            edmd::predictPairContact(flight.trajectory, other.trajectory, t, limit);
        if (contact.found) {
            Prediction prediction;
            prediction.time = contact.time;
            prediction.idA = id;
            prediction.idB = particles[j].getId();
            out.push_back(prediction);
        }
    }
}

void SimulationHybrid::enqueue(const std::vector<Prediction>& predicted) {
    for (Prediction prediction : predicted) {
        prediction.versionA = version[static_cast<std::size_t>(prediction.idA)];
        if (!prediction.wall) {
            prediction.versionB = version[static_cast<std::size_t>(prediction.idB)];
        }
        predictions.push(prediction);
    }
}

// ---------------------------------------------------------------------------

int SimulationHybrid::getPopulation() const {
    int population = 0;
    for (const types::Particle& particle : particles) {
        if (particle.getPosition().y > 0) {
            ++population;
        }
    }
    return population;
}

int SimulationHybrid::getTimeDrivenCount() const {
    int count = 0;
    for (const types::Particle& particle : particles) {
        if (particle.getScheme() == types::MotionScheme::TimeDriven) {
            ++count;
        }
    }
    return count;
}

}
