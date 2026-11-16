#include "simulation/SimulationEDMD.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>

namespace simulation {

using edmd::Phase;

SimulationEDMD::SimulationEDMD(int particleCount, unsigned seed, types::Board board,
                               NeighbourSearch search, double rebuildInterval)
    : board(board), search(search), rebuildInterval(rebuildInterval),
      grid(board.getSide(), neighbourReach(rebuildInterval)) {
    initializeAgents(particleCount, seed);
    flags.assign(agents.size(), 0);
    horizon.assign(agents.size(), edmd::infinity);
    for (std::size_t i = 0; i < agents.size(); ++i) {
        horizon[i] = validUntil(i);
        everyone.push_back(i);
    }
    scheduleAll();
    materialize();
}

double SimulationEDMD::neighbourReach(double interval) {
    const double maxSpeed = std::max(cpm::desiredSpeed, cpm::escapeSpeed);
    const double closingSpeed = 2 * maxSpeed + 2 * edmd::growthRate;   // 4 m/s
    // Margen mínimo por los eventos agrupados apenas después de la reconstrucción.
    return 2 * cpm::maxRadius + closingSpeed * interval + 1e-6;
}

// Mismo sorteo que Simulation::initializeParticles: con la misma semilla, los dos
// esquemas parten del mismo estado.
void SimulationEDMD::initializeAgents(int particleCount, unsigned seed) {
    std::mt19937 random(seed);
    std::uniform_real_distribution<double> coordinate(cpm::maxRadius, board.getSide() - cpm::maxRadius);
    agents.reserve(static_cast<std::size_t>(particleCount));

    while (agents.size() < static_cast<std::size_t>(particleCount)) {
        const types::Vector2D position{coordinate(random), coordinate(random)};
        bool overlaps = false;
        for (const Agent& agent : agents) {
            if ((position - agent.trajectory.x0).norm() <= 2 * cpm::maxRadius) {
                overlaps = true;
                break;
            }
        }
        if (overlaps) {
            continue;
        }
        Agent agent;
        agent.id = static_cast<int>(agents.size());
        agent.target = board.getDoorTarget();
        agent.trajectory.phase = Phase::Free;
        agent.trajectory.t0 = 0.0;
        agent.trajectory.x0 = position;
        agent.trajectory.direction = (agent.target - position).normalized();
        agents.push_back(agent);
    }
}

void SimulationEDMD::step() {
    advanceTo(time + cpm::dt);
}

void SimulationEDMD::advanceTo(double targetTime) {
    while (true) {
        const double eventTime = nextEventTime();
        if (std::min(eventTime, nextRebuild) > targetTime) {
            break;
        }
        // Los eventos de la ventana que termina se procesan antes de reconstruir.
        if (eventTime <= nextRebuild) {
            processNextGroup();
        } else {
            rebuildNeighbours(nextRebuild);
        }
    }
    time = targetTime;
    materialize();
}

void SimulationEDMD::materialize() {
    particles.clear();
    for (const Agent& agent : agents) {
        if (!agent.active) {
            continue;
        }
        types::Particle particle(agent.id, agent.trajectory.positionAt(time), agent.trajectory.radiusAt(time));
        particle.setVelocity(agent.trajectory.velocityAt(time));
        particle.setTarget(agent.target);
        particle.setScheme(types::MotionScheme::EventDriven);
        particles.push_back(particle);
    }
}

int SimulationEDMD::getPopulation() const {
    int population = 0;
    for (const types::Particle& particle : particles) {
        if (particle.getPosition().y > 0) {
            ++population;
        }
    }
    return population;
}

// ---------------------------------------------------------------------------
// Agenda
// ---------------------------------------------------------------------------

bool SimulationEDMD::isCurrent(const Event& event) const {
    const Agent& a = agents[event.i];
    if (!a.active || a.trajectory.version != event.versionI) {
        return false;
    }
    if (event.type == EventType::PairContact) {
        const Agent& b = agents[event.j];
        if (!b.active || b.trajectory.version != event.versionJ) {
            return false;
        }
    }
    return true;
}

// Descarta los eventos obsoletos del frente y devuelve el instante del próximo vigente.
double SimulationEDMD::nextEventTime() {
    while (!queue.empty() && !isCurrent(queue.top())) {
        queue.pop();
        ++staleEvents;
    }
    return queue.empty() ? edmd::infinity : queue.top().time;
}

void SimulationEDMD::processNextGroup() {
    // Grupo: todos los eventos vigentes a menos de groupTolerance del primero.
    std::vector<Event> group;
    const double first = queue.top().time;
    double groupTime = first;
    while (!queue.empty()) {
        const Event& top = queue.top();
        if (!isCurrent(top)) {
            queue.pop();
            ++staleEvents;
            continue;
        }
        if (top.time > first + edmd::groupTolerance) {
            break;
        }
        groupTime = std::max(groupTime, top.time);
        group.push_back(top);
        queue.pop();
        ++processedEvents;
    }

    // Detector de cascadas (colapso de Zenón): eventos cada vez más seguidos sin
    // que avance el tiempo. Mejor un error claro que una corrida colgada sin memoria.
    if (groupTime - progressTime > edmd::progressInterval) {
        progressTime = groupTime;
        groupsWithoutProgress = 0;
    } else if (++groupsWithoutProgress > edmd::maxGroupsWithoutProgress) {
        throw std::runtime_error("EDMD: event cascade without time progress at t = " +
                                 std::to_string(groupTime) + " s.");
    }

    modified.clear();
    processGroup(group, groupTime);
    clock = groupTime;

    // Solo se reprograman los agentes activos cuyo tramo cambió (los que registró
    // bumpVersion). Primero se actualizan todos sus horizontes, porque cada par usa
    // los de ambos. Ningún paso recorre a los N agentes.
    std::vector<std::size_t> changedList;
    for (std::size_t i : modified) {
        if (agents[i].active) {
            setFlag(i, changedFlag);
            changedList.push_back(i);
            horizon[i] = validUntil(i);
        }
    }
    std::sort(changedList.begin(), changedList.end());
    for (std::size_t i : changedList) {
        scheduleChanged(i);
    }
    clearFlags();
}

// Todo cambio de tramo pasa por acá: invalida los eventos del agente y lo
// registra para reprogramarlo al terminar el grupo.
void SimulationEDMD::bumpVersion(std::size_t i) {
    ++agents[i].trajectory.version;
    if ((flags[i] & modifiedFlag) == 0) {
        setFlag(i, modifiedFlag);
        modified.push_back(i);
    }
}

void SimulationEDMD::setFlag(std::size_t i, unsigned char flag) {
    if (flags[i] == 0) {
        flagged.push_back(i);
    }
    flags[i] = static_cast<unsigned char>(flags[i] | flag);
}

void SimulationEDMD::clearFlags() {
    for (std::size_t i : flagged) {
        flags[i] = 0;
    }
    flagged.clear();
}

void SimulationEDMD::enqueue(const std::vector<Event>& events) {
    for (Event event : events) {
        event.versionI = agents[event.i].trajectory.version;
        if (event.type == EventType::PairContact) {
            event.versionJ = agents[event.j].trajectory.version;
        }
        queue.push(event);
    }
}

void SimulationEDMD::scheduleAll() {
    std::vector<Event> events;
    for (std::size_t i = 0; i < agents.size(); ++i) {
        addOwnEvents(i, events);
        addWallEvents(i, events);
    }
    enqueue(events);

    if (search == NeighbourSearch::CellGrid) {
        rebuildNeighbours(0.0);
        return;
    }
    events.clear();
    for (std::size_t i = 0; i < agents.size(); ++i) {
        for (std::size_t j = i + 1; j < agents.size(); ++j) {
            addPairEvent(i, j, events);
        }
    }
    enqueue(events);
}

// Si los dos agentes de un par cambiaron, el par se programa una sola vez.
void SimulationEDMD::scheduleChanged(std::size_t i) {
    std::vector<Event> events;
    addOwnEvents(i, events);
    addWallEvents(i, events);
    for (std::size_t j : partnersOf(i)) {
        const bool otherChanged = (flags[j] & changedFlag) != 0;
        if (j == i || !agents[j].active || (otherChanged && j < i)) {
            continue;
        }
        addPairEvent(i, j, events);
    }
    enqueue(events);
}

void SimulationEDMD::addOwnEvents(std::size_t i, std::vector<Event>& out) const {
    const edmd::Trajectory& trajectory = agents[i].trajectory;
    Event event;
    event.i = i;
    if (trajectory.phase == Phase::Escape) {
        event.type = EventType::EscapeEnd;
        event.time = trajectory.phaseEnd();
        out.push_back(event);
    }
    if (trajectory.phase == Phase::Recovery) {
        event.type = EventType::RecoveryEnd;
        event.time = trajectory.phaseEnd();
        out.push_back(event);
    }
    const double door = doorCrossTime(i);
    if (door != edmd::infinity) {
        event.type = EventType::DoorCross;
        event.time = door;
        out.push_back(event);
    }
    const double removal = removalTime(i);
    if (removal != edmd::infinity) {
        event.type = EventType::Removal;
        event.time = removal;
        out.push_back(event);
    }
}

// Las paredes son pocas y fijas: no dependen de la grilla ni de su ventana.
void SimulationEDMD::addWallEvents(std::size_t i, std::vector<Event>& out) const {
    const std::array<types::Wall, 5>& walls = board.getWalls();
    for (std::size_t w = 0; w < walls.size(); ++w) {
        const edmd::ContactPrediction contact =
            edmd::predictWallContact(agents[i].trajectory, walls[w], clock, horizon[i]);
        if (contact.found) {
            Event event;
            event.type = EventType::WallContact;
            event.time = contact.time;
            event.i = i;
            event.j = w;
            out.push_back(event);
        }
    }
}

void SimulationEDMD::addPairEvent(std::size_t i, std::size_t j, std::vector<Event>& out) const {
    const edmd::ContactPrediction contact =
        edmd::predictPairContact(agents[i].trajectory, agents[j].trajectory, clock, pairHorizon(i, j));
    if (contact.found) {
        Event event;
        event.type = EventType::PairContact;
        event.time = contact.time;
        event.i = i;
        event.j = j;
        out.push_back(event);
    }
}

// ---------------------------------------------------------------------------
// Vecinos
// ---------------------------------------------------------------------------

// Reconstruye la grilla con las posiciones en t y programa los choques de la
// nueva ventana [t, t + rebuildInterval]. No hace falta invalidar nada: ningún
// choque de la ventana anterior se predijo más allá de t.
void SimulationEDMD::rebuildNeighbours(double t) {
    clock = t;
    nextRebuild = t + rebuildInterval;
    ++rebuilds;

    std::vector<types::Particle> snapshot;
    std::vector<std::size_t> owner;
    for (std::size_t i = 0; i < agents.size(); ++i) {
        if (agents[i].active) {
            snapshot.emplace_back(agents[i].id, agents[i].trajectory.positionAt(t),
                                  agents[i].trajectory.radiusAt(t));
            owner.push_back(i);
        }
    }
    grid.rebuild(snapshot);

    partners.assign(agents.size(), std::vector<std::size_t>());
    // La grilla devuelve un cuadrado de 3 x 3 celdas; el alcance es un círculo.
    // Se descartan los que ya están más lejos que el alcance: no pueden tocarse
    // antes de la próxima reconstrucción (unas 3 veces menos candidatos).
    const double reach = neighbourReach(rebuildInterval);
    for (std::size_t k = 0; k < snapshot.size(); ++k) {
        std::vector<std::size_t>& list = partners[owner[k]];
        const types::Vector2D position = snapshot[k].getPosition();
        for (std::size_t c : grid.getCandidates(position)) {
            if (c != k && (snapshot[c].getPosition() - position).norm() <= reach) {
                list.push_back(owner[c]);
            }
        }
        // Mismo orden que AllPairs: las direcciones de escape se suman en el mismo
        // orden y los dos modos coinciden hasta el último decimal mientras no diverjan.
        std::sort(list.begin(), list.end());
    }

    std::vector<Event> events;
    for (std::size_t i : owner) {
        for (std::size_t j : partners[i]) {
            if (j > i) {
                addPairEvent(i, j, events);
            }
        }
    }
    enqueue(events);
}

const std::vector<std::size_t>& SimulationEDMD::partnersOf(std::size_t i) const {
    return search == NeighbourSearch::CellGrid ? partners[i] : everyone;
}

// Un choque entre agentes no se predice más allá del fin de ninguno de los dos
// tramos ni de la próxima reconstrucción de la grilla (infinito en AllPairs).
double SimulationEDMD::pairHorizon(std::size_t i, std::size_t j) const {
    return std::min({horizon[i], horizon[j], nextRebuild});
}

// ---------------------------------------------------------------------------
// Horizontes: hasta cuándo valen las fórmulas del tramo actual
// ---------------------------------------------------------------------------

double SimulationEDMD::doorCrossTime(std::size_t i) const {
    const Agent& agent = agents[i];
    if (agent.crossed) {
        return edmd::infinity;
    }
    const double t = edmd::timeToReachY(agent.trajectory, 0.0, clock);
    if (t == edmd::infinity) {
        return edmd::infinity;
    }
    const double crossingX = agent.trajectory.positionAt(t).x;
    if (std::abs(crossingX - board.getSide() / 2) > board.getDoorWidth() / 2) {
        return edmd::infinity;
    }
    return t;
}

double SimulationEDMD::removalTime(std::size_t i) const {
    return edmd::timeToReachY(agents[i].trajectory, board.getExitTarget().y, clock);
}

double SimulationEDMD::validUntil(std::size_t i) const {
    return std::min({agents[i].trajectory.phaseEnd(), doorCrossTime(i), removalTime(i)});
}

// ---------------------------------------------------------------------------
// Reglas CPM
// ---------------------------------------------------------------------------

void SimulationEDMD::processGroup(const std::vector<Event>& group, double t) {
    // 1) Cambios que no alteran posiciones ni radios en t: retiro, fin de
    //    recuperación y cruce de puerta.
    for (const Event& event : group) {
        if (event.type == EventType::Removal && agents[event.i].active) {
            agents[event.i].active = false;
            bumpVersion(event.i);
        }
    }
    for (const Event& event : group) {
        if (event.type == EventType::RecoveryEnd && agents[event.i].active) {
            startFree(event.i, t);
        }
    }
    for (const Event& event : group) {
        if (event.type != EventType::DoorCross || !agents[event.i].active) {
            continue;
        }
        Agent& agent = agents[event.i];
        agent.crossed = true;
        agent.target = board.getExitTarget();
        evacuationTime = t;
        if (agent.trajectory.phase == Phase::Free) {
            startFree(event.i, t);
        } else if (agent.trajectory.phase == Phase::Recovery) {
            redirectRecovery(event.i, t);
        } else {
            // En escape la dirección se mantiene, pero el horizonte ya no termina
            // en la puerta: hay que recalcular sus eventos.
            bumpVersion(event.i);
        }
    }

    // 2) Agentes afectados por contactos o por fin de escape.
    std::vector<std::size_t> affected;
    for (const Event& event : group) {
        if (!agents[event.i].active) {
            continue;
        }
        if (event.type == EventType::PairContact && agents[event.j].active) {
            setFlag(event.i, contactFlag);
            setFlag(event.j, contactFlag);
            addAffected(affected, event.i);
            addAffected(affected, event.j);
        } else if (event.type == EventType::WallContact) {
            setFlag(event.i, contactFlag);
            addAffected(affected, event.i);
        } else if (event.type == EventType::EscapeEnd) {
            setFlag(event.i, escapeEndFlag);
            addAffected(affected, event.i);
        }
    }

    // 3) Foto: todos los contactos se calculan ANTES de contraer a nadie.
    //    Quien toca a un afectado también se reevalúa (el contacto es mutuo).
    //    sets[k] corresponde a affected[k].
    std::vector<ContactSet> sets;
    for (std::size_t k = 0; k < affected.size(); ++k) {
        sets.push_back(contactSetAt(affected[k], t));
        for (std::size_t j : sets[k].touching) {
            addAffected(affected, j);
        }
    }

    // 4) Aplicar.
    for (std::size_t k = 0; k < affected.size(); ++k) {
        const std::size_t i = affected[k];
        if (!sets[k].points.empty()) {
            startEscape(i, sets[k].points, t);
        } else if ((flags[i] & escapeEndFlag) != 0) {
            startRecovery(i, t);
        } else if ((flags[i] & contactFlag) != 0) {
            // La predicción no se cumplió: se recalculan sus eventos.
            ++phantomContacts;
            bumpVersion(i);
        }
    }
}

void SimulationEDMD::addAffected(std::vector<std::size_t>& affected, std::size_t i) {
    if ((flags[i] & affectedFlag) == 0) {
        setFlag(i, affectedFlag);
        affected.push_back(i);
    }
}

SimulationEDMD::ContactSet SimulationEDMD::contactSetAt(std::size_t i, double t) const {
    ContactSet set;
    const edmd::Trajectory& trajectory = agents[i].trajectory;
    const types::Vector2D position = trajectory.positionAt(t);
    // Quien toca a i en t estaba dentro del alcance en la última reconstrucción.
    for (std::size_t j : partnersOf(i)) {
        if (j == i || !agents[j].active) {
            continue;
        }
        if (edmd::pairGap(trajectory, agents[j].trajectory, t) <= edmd::contactTolerance) {
            set.touching.push_back(j);
            set.points.push_back(agents[j].trajectory.positionAt(t));
        }
    }
    for (const types::Wall& wall : board.getWalls()) {
        if (edmd::wallGap(trajectory, wall, t) <= edmd::contactTolerance) {
            set.points.push_back(wall.closestPoint(position));
        }
    }
    return set;
}

void SimulationEDMD::startFree(std::size_t i, double t) {
    edmd::Trajectory& trajectory = agents[i].trajectory;
    const types::Vector2D position = trajectory.positionAt(t);
    trajectory.phase = Phase::Free;
    trajectory.t0 = t;
    trajectory.x0 = position;
    trajectory.direction = (agents[i].target - position).normalized();
    bumpVersion(i);
}

void SimulationEDMD::startRecovery(std::size_t i, double t) {
    edmd::Trajectory& trajectory = agents[i].trajectory;
    const types::Vector2D position = trajectory.positionAt(t);
    trajectory.phase = Phase::Recovery;
    trajectory.t0 = t;
    trajectory.x0 = position;
    trajectory.direction = (agents[i].target - position).normalized();
    trajectory.recoveryStart = t;
    bumpVersion(i);
}

// Recuperación en curso que cambia de target: no vuelve a rmin.
void SimulationEDMD::redirectRecovery(std::size_t i, double t) {
    edmd::Trajectory& trajectory = agents[i].trajectory;
    const types::Vector2D position = trajectory.positionAt(t);
    trajectory.t0 = t;
    trajectory.x0 = position;
    trajectory.direction = (agents[i].target - position).normalized();
    bumpVersion(i);
}

void SimulationEDMD::startEscape(std::size_t i, const std::vector<types::Vector2D>& points, double t) {
    edmd::Trajectory& trajectory = agents[i].trajectory;
    const types::Vector2D position = trajectory.positionAt(t);
    types::Vector2D sum;
    for (const types::Vector2D& point : points) {
        sum += (position - point).normalized();
    }
    trajectory.phase = Phase::Escape;
    trajectory.t0 = t;
    trajectory.x0 = position;
    trajectory.direction = sum.normalized();
    bumpVersion(i);
}

}
