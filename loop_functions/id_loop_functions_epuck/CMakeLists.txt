add_library(id_loop_functions_epuck MODULE
  id_qtuser_functions_epuck.h
  id_qtuser_functions_epuck.cpp)

target_link_libraries(id_loop_functions_epuck
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_epuck
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
