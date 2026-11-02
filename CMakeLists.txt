cmake_minimum_required(VERSION 3.10)
project(SdS_TP5 LANGUAGES CXX)

add_library(tp5_core
    lib/types/Board.cpp
    lib/io/StateWriter.cpp
    lib/simulation/Simulation.cpp
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
