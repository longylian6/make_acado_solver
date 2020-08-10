cmake_minimum_required(VERSION 3.10)
project(test_on_clion)

set(CMAKE_CXX_STANDARD 11)

add_library(nmpc_lib
        solver/qpoases/SRC/Bounds.cpp
        solver/qpoases/SRC/Constraints.cpp
        solver/qpoases/SRC/CyclingManager.cpp
        solver/qpoases/SRC/Indexlist.cpp
        solver/qpoases/SRC/MessageHandling.cpp
        solver/qpoases/SRC/QProblem.cpp
        solver/qpoases/SRC/QProblemB.cpp
        solver/qpoases/SRC/SubjectTo.cpp
        solver/qpoases/SRC/Utils.cpp
        solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
        solver/OCPexport/acado_qpoases_interface.cpp
        solver/OCPexport/acado_integrator.c
        solver/OCPexport/acado_solver.c
        solver/OCPexport/acado_auxiliary_functions.c
)

target_include_directories(nmpc_lib PUBLIC
        solver/
        solver/OCPexport
        solver/qpoases
        solver/qpoases/INCLUDE
        solver/qpoases/SRC
        )

add_executable(main main.cpp)

target_link_libraries(main nmpc_lib)

# add_dependencies(nonlinear_mpc_node ${${PROJECT_NAME}_EXPORTED_TARGETS})