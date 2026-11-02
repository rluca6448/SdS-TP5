#pragma once

#include "types/Particle.h"

#include <fstream>
#include <string>
#include <vector>

namespace io {

class StateWriter {
public:
    explicit StateWriter(const std::string& path);
    void writeState(const std::vector<types::Particle>& particles, double time);

private:
    std::ofstream out;
};

}
