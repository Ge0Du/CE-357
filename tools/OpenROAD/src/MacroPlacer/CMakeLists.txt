add_library(MacroPlacer STATIC
  MacroPlacer.cpp
  MacroPlacerPlugin.cpp
)

target_include_directories(MacroPlacer
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(MacroPlacer
  PUBLIC
    odb
)
