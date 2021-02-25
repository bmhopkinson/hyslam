

HYSLAM_ADD_SOURCES(
  PnPsolver.h PnPsolver.cc
  Sim3Solver.h Sim3Solver.cc
        Triangulator.h  Triangulator.cpp
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
