add_library(fault_detection SHARED fault_detection_multifault.h fault_detection_multifault.cpp fault_detection.h fault_detection.cpp ../util/util.h ../util/util.cpp ../behaviors/behavior.h ../behaviors/behavior.cpp ../behaviors/aggregatebehavior.h ../behaviors/aggregatebehavior.cpp ../behaviors/dispersebehavior.h ../behaviors/dispersebehavior.cpp ../behaviors/randomwalkbehavior.h ../behaviors/randomwalkbehavior.cpp ../behaviors/flockingbehavior.h ../behaviors/flockingbehavior.cpp ../behaviors/homingbehavior.h ../behaviors/homingbehavior.cpp )


find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(fault_detection
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot
  xgboost
  ${PYTHON_LIBRARIES})
