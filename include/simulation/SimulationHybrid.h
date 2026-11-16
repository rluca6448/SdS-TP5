#pragma once

#include "neighbours/Grid.h"
#include "simulation/CpmParameters.h"
#include "simulation/EventPrediction.h"
#include "types/Board.h"
#include "types/Particle.h"

#include <algorithm>
#include <cstddef>
#include <queue>
#include <vector>

namespace simulation {

// Híbrido ED + TS con una sola bolsa TS y el reloj común del CPM (dt = 1/30 s).
//  - TS: agentes en contacto, recuperándose, con un choque previsto dentro de
//    k * dt, o a distancia de cierre d* o menos de otro agente TS. Siguen
//    exactamente las reglas del TSDMD.
//  - ED: el resto. Vuelan en línea recta a vd hacia su target y sus choques
//    futuros se predicen; no se les buscan contactos tick a tick.
// Cambiar de esquema no contrae, mueve ni reorienta a nadie: solo cambia cómo
// se calcula. Por eso la física es la del TSDMD para cualquier k >= 0 y d* >= safeDistance.
class SimulationHybrid {
public:
    static constexpr int defaultAnticipation = 1;

    // Distancia de cierre mínima: si un agente ED está a esta distancia o menos de
    // uno TS, debe entrar a TS, porque podrían tocarse durante el próximo paso.
    // La distancia de cierre d* es configurable, pero nunca menor que esta.
    static constexpr double safeDistance =
        2 * cpm::maxRadius + 2 * std::max(cpm::desiredSpeed, cpm::escapeSpeed) * cpm::dt;   // 0,60 m

    SimulationHybrid(int particleCount = 500, unsigned seed = 42, types::Board board = {},
                     int anticipation = defaultAnticipation, bool forceTimeDriven = false,
                     double closureDistance = safeDistance);

    void step();

    const types::Board& getBoard() const { return board; }
    const std::vector<types::Particle>& getParticles() const { return particles; }
    double getTime() const { return time; }
    double getDt() const { return cpm::dt; }
    int getPopulation() const;
    double getEvacuationTime() const { return evacuationTime; }

    // Diagnóstico
    int getTimeDrivenCount() const;
    long getPredictedEntries() const { return predictedEntries; }
    long getBoundaryEntries() const { return boundaryEntries; }
    long getPairPredictions() const { return pairPredictions; }

    // Verificación: cuenta, en cada tick, contactos en los que participa algún
    // agente ED. Deben ser cero. Cuesta O(N) por tick; desactivado por defecto.
    void setCheckMissedContacts(bool value) { checkMissedContacts = value; }
    long getMissedContacts() const { return missedContacts; }

private:
    // Vuelo libre de un agente ED, válido hasta flightLimit (el fin del tick en
    // que cruza la puerta o se retira, como en el TSDMD).
    struct Flight {
        edmd::Trajectory trajectory;
        double limit = edmd::infinity;
    };

    // Choque previsto entre dos agentes ED (o un agente ED y una pared), con las
    // versiones de sus vuelos al predecirse. Se identifican por id: el vector de
    // partículas se compacta al retirar agentes.
    struct Prediction {
        double time = 0.0;
        int idA = 0;
        int idB = 0;            // otro agente, o índice de pared si wall
        bool wall = false;
        int versionA = 0;
        int versionB = 0;
    };

    struct LaterFirst {
        bool operator()(const Prediction& a, const Prediction& b) const { return a.time > b.time; }
    };

    void initializeParticles(int particleCount, unsigned seed);

    // Paso: clasificar, aplicar reglas, mover, puerta y retiro.
    void classify(std::vector<char>& timeDriven);
    void markTimeDriven(std::size_t i, std::vector<char>& timeDriven, std::vector<std::size_t>& members,
                        std::vector<char>& flying, long& counter);
    void includeNeighbours(std::vector<char>& timeDriven, std::vector<std::size_t>& members,
                           std::size_t start, std::vector<char>& flying);
    bool touchesSomething(std::size_t i) const;
    void findContacts(const std::vector<char>& timeDriven, std::vector<bool>& contacts,
                      std::vector<types::Vector2D>& directions) const;
    void countMissedContacts(const std::vector<char>& timeDriven);

    // Vuelos y predicciones
    Flight flightFrom(std::size_t i, double t) const;
    double tickAtOrAfter(double t) const;
    bool isCurrent(const Prediction& prediction) const;
    void predictFlight(std::size_t i, const Flight& flight, double t, const std::vector<char>& flying,
                       const std::vector<char>& leaving, const std::vector<Flight>& tentative,
                       std::vector<Prediction>& out);
    void enqueue(const std::vector<Prediction>& predicted);
    bool isEventDriven(std::size_t i) const {
        return particles[i].getScheme() == types::MotionScheme::EventDriven;
    }

    types::Board board;
    int anticipation;
    double closureDistance;             // d*: entra a TS quien esté a d* o menos de un TS
    bool forceTimeDriven;
    neighbours::Grid contactGrid;       // celdas de 2 rmax, igual que el TSDMD
    neighbours::Grid boundaryGrid;      // celdas de closureDistance, para el cierre de la bolsa
    std::vector<types::Particle> particles;
    std::vector<Flight> flights;        // paralelo a particles; vale para los agentes ED
    std::vector<int> indexOf;           // id -> índice en particles, o -1 si se retiró
    std::vector<int> version;           // por id: cambia cuando cambia o termina su vuelo
    std::priority_queue<Prediction, std::vector<Prediction>, LaterFirst> predictions;
    double time = 0.0;
    double evacuationTime = 0.0;
    long predictedEntries = 0;
    long boundaryEntries = 0;
    long pairPredictions = 0;
    bool checkMissedContacts = false;
    long missedContacts = 0;
};

}
