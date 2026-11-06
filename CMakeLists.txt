cmake_minimum_required(VERSION 3.10)
project(SdS_TP5 LANGUAGES CXX)

add_library(tp5_core
    lib/types/Board.cpp
    lib/io/StateWriter.cpp
    lib/simulation/Simulation.cpp
    lib/simulation/EventPrediction.cpp
    lib/simulation/SimulationEDMD.cpp
    lib/simulation/SimulationHybrid.cpp
    lib/neighbours/Grid.cpp
)
target_include_directories(tp5_core PUBLIC include)
target_compile_features(tp5_core PUBLIC cxx_std_17)

add_executable(tp5 src/main.cpp)
target_link_libraries(tp5 PRIVATE tp5_core)

if(MSVC)
    target_compile_options(tp5_core PRIVATE /W4)
    target_compile_options(tp5 PRIVATE /W4)
else()
    target_compile_options(tp5_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(tp5 PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Programas de verificación del EDMD (no forman parte de la entrega).
option(TP5_BUILD_VERIFICATION "Compilar los programas de verificación del EDMD" OFF)
if(TP5_BUILD_VERIFICATION)
    add_executable(test_edmd_prototype tests/test_edmd.cpp)
    target_compile_features(test_edmd_prototype PRIVATE cxx_std_17)

    add_executable(verify_edmd tests/verify_integration.cpp)
    target_include_directories(verify_edmd PRIVATE tests)
    target_link_libraries(verify_edmd PRIVATE tp5_core)

    add_executable(overlap_diagnostic tests/overlap_diagnostic.cpp)
    target_link_libraries(overlap_diagnostic PRIVATE tp5_core)

    add_executable(benchmark_edmd tests/benchmark_edmd.cpp)
    target_link_libraries(benchmark_edmd PRIVATE tp5_core)

    add_executable(verify_hybrid tests/verify_hybrid.cpp)
    target_link_libraries(verify_hybrid PRIVATE tp5_core)
endif()
