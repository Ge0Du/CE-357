cmake_minimum_required(VERSION 3.13)
project(MacroPlacerPlugin)

# ─────────── Gather sources ───────────
set(MACRO_PLACER_SOURCES
  MacroPlacer.cpp
  MacroPlacerCmd.cpp
)

# ─────────── Build a static library ───────────
# Rename the target to openroad_macroplacer
add_library(openroad_macroplacer STATIC
  ${MACRO_PLACER_SOURCES}
)

# ─────────── Include directories ───────────
#  1. ${CMAKE_CURRENT_SOURCE_DIR}       → for MacroPlacer.h
#  2. ${CMAKE_SOURCE_DIR}/include       → for <ord/OpenRoad.hh> and <odb/db.h>
#  3. ${CMAKE_SOURCE_DIR}/src           → for any OpenROAD internal headers
target_include_directories(openroad_macroplacer
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}       # MacroPlacer.h is here
    ${CMAKE_SOURCE_DIR}/include       # so <ord/OpenRoad.hh> and <odb/db.h> resolve
    ${CMAKE_SOURCE_DIR}/src           # for other OpenROAD internals, if needed
)

# ─────────── Link dependencies ───────────
# MacroPlacer only depends on OpenDB (the built-in `odb` target), so link that.
target_link_libraries(openroad_macroplacer
  PUBLIC
    odb
)
