#pragma once

#include "neighbours/Grid.h"
#include "simulation/CpmParameters.h"
#include "simulation/EventPrediction.h"
#include "types/Board.h"
#include "types/Particle.h"

#include <cstddef>
#include <queue>
#include <vector>

namespace simulation {

// Cómo se buscan los posibles compañeros de choque de un agente.
//  - AllPairs: contra todos los demás (O(N) por agente modificado). Referencia.
//  - CellGrid: la grilla se reconstruye cada rebuildInterval y ningún choque se
//    predice más allá de la próxima reconstrucción; así alcanza con mirar los
//    agentes que en la reconstrucción estaban a menos del alcance máximo.
enum class NeighbourSearch { AllPairs, CellGrid };

// EDMD puro para CPM (alternativa B): contactos resueltos en tiempo continuo,
// escape de duración dt y recuperación continua con dr/dt = rmax / tau.
// Misma interfaz pública que Simulation, para que main pueda usar cualquiera.
class SimulationEDMD {
public:
    static constexpr double defaultRebuildInterval = 0.05;   // [s] ver tests/benchmark_edmd (interval)

    SimulationEDMD(int particleCount = 500, unsigned seed = 42, types::Board board = {},
                   NeighbourSearch search = NeighbourSearch::CellGrid,
                   double rebuildInterval = defaultRebuildInterval);

    // Procesa todos los eventos hasta getTime() + getDt() y deja las partículas
    // calculadas en ese instante. No es un paso de CPM: es el intervalo de muestreo,
    // igual al dt del TSDMD para que las dos salidas tengan los mismos instantes.
    void step();
    void advanceTo(double targetTime);

    const types::Board& getBoard() const { return board; }
    const std::vector<types::Particle>& getParticles() const { return particles; }
    double getTime() const { return time; }
    double getDt() const { return cpm::dt; }
    int getPopulation() const;
    double getEvacuationTime() const { return evacuationTime; }

    // Diagnóstico de la agenda.
    long getProcessedEvents() const { return processedEvents; }
    long getStaleEvents() const { return staleEvents; }
    long getPhantomContacts() const { return phantomContacts; }
    long getRebuilds() const { return rebuilds; }

    // Distancia entre centros a partir de la cual dos agentes no pueden tocarse
    // antes de que pase el intervalo dado.
    static double neighbourReach(double interval);

private:
    enum class EventType { PairContact, WallContact, EscapeEnd, RecoveryEnd, DoorCross, Removal };

    // Un evento guarda la versión de sus agentes al predecirse. Si alguna cambió
    // antes de procesarlo, es obsoleto y se descarta al sacarlo de la agenda.
    struct Event {
        double time = 0.0;
        EventType type = EventType::EscapeEnd;
        std::size_t i = 0;
        std::size_t j = 0;          // otro agente (PairContact) o pared (WallContact)
        int versionI = 0;
        int versionJ = 0;
    };

    struct LaterFirst {
        bool operator()(const Event& a, const Event& b) const { return a.time > b.time; }
    };

    // Los agentes nunca se borran: el índice es un identificador estable para la agenda.
    struct Agent {
        int id = 0;
        edmd::Trajectory trajectory;
        types::Vector2D target;
        bool crossed = false;
        bool active = true;
    };

    struct ContactSet {
        std::vector<std::size_t> touching;
        std::vector<types::Vector2D> points;
    };

    void initializeAgents(int particleCount, unsigned seed);
    void materialize();

    // Agenda
    double nextEventTime();
    void processNextGroup();
    bool isCurrent(const Event& event) const;
    void enqueue(const std::vector<Event>& events);
    void scheduleAll();
    void scheduleChanged(std::size_t i);

    // Marcas por agente, válidas durante un grupo. Se borran solo las usadas,
    // para que procesar un grupo no cueste O(N).
    static constexpr unsigned char contactFlag = 1;
    static constexpr unsigned char escapeEndFlag = 2;
    static constexpr unsigned char affectedFlag = 4;
    static constexpr unsigned char modifiedFlag = 8;
    static constexpr unsigned char changedFlag = 16;
    void bumpVersion(std::size_t i);
    void setFlag(std::size_t i, unsigned char flag);
    void clearFlags();
    void addAffected(std::vector<std::size_t>& affected, std::size_t i);
    void addOwnEvents(std::size_t i, std::vector<Event>& out) const;
    void addWallEvents(std::size_t i, std::vector<Event>& out) const;
    void addPairEvent(std::size_t i, std::size_t j, std::vector<Event>& out) const;

    // Vecinos
    void rebuildNeighbours(double t);
    const std::vector<std::size_t>& partnersOf(std::size_t i) const;
    double pairHorizon(std::size_t i, std::size_t j) const;

    // Horizontes de validez de cada tramo
    double doorCrossTime(std::size_t i) const;
    double removalTime(std::size_t i) const;
    double validUntil(std::size_t i) const;

    // Reglas CPM
    void processGroup(const std::vector<Event>& group, double t);
    ContactSet contactSetAt(std::size_t i, double t) const;
    void startFree(std::size_t i, double t);
    void startRecovery(std::size_t i, double t);
    void redirectRecovery(std::size_t i, double t);
    void startEscape(std::size_t i, const std::vector<types::Vector2D>& points, double t);

    types::Board board;
    NeighbourSearch search;
    double rebuildInterval;
    neighbours::Grid grid;
    std::vector<Agent> agents;
    std::vector<double> horizon;                       // validUntil de cada agente, en caché
    std::vector<std::vector<std::size_t>> partners;    // CellGrid: candidatos de cada agente
    std::vector<std::size_t> everyone;                 // AllPairs: todos los índices
    std::vector<unsigned char> flags;                  // marcas del grupo en curso
    std::vector<std::size_t> flagged;                  // agentes con alguna marca
    std::vector<std::size_t> modified;                 // agentes cuyo tramo cambió en el grupo
    std::vector<types::Particle> particles;            // vista en getTime(), solo agentes activos
    std::priority_queue<Event, std::vector<Event>, LaterFirst> queue;
    double time = 0.0;               // instante de muestreo
    double clock = 0.0;              // instante del último grupo de eventos procesado
    double nextRebuild = edmd::infinity;
    double progressTime = 0.0;        // detector de cascadas
    long groupsWithoutProgress = 0;
    double evacuationTime = 0.0;
    long processedEvents = 0;
    long staleEvents = 0;
    long phantomContacts = 0;
    long rebuilds = 0;
};

}
