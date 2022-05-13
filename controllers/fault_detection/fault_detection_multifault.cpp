/* Include the controller definition */
#include "fault_detection_multifault.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <random>
#include <vector>

/****************************************/
/****************************************/

CEPuckFaultDetectionMultiFault::CEPuckFaultDetectionMultiFault()
    : CEPuckFaultDetection() {}

/****************************************/
/****************************************/

bool CEPuckFaultDetectionMultiFault::check_fault(int id,
                                                 std::string fault) const {
  std::string real_fault;
  auto it = CConfiguration::FAULTY_ROBOTS_MAP.find(id);
  if (it != CConfiguration::FAULTY_ROBOTS_MAP.cend()) {
    real_fault = (*it).second;
  } else {
    real_fault = CConfiguration::FAULT_NONE;
  }
  return (fault.compare(real_fault) == 0);
}

std::string CEPuckFaultDetectionMultiFault::generate_output_filename() const {
  int random_seed = CSimulator::GetInstance().GetRandomSeed();
  std::string binary_or_numerical = "numerical";
  if (CConfiguration::BOOLEAN_OBSERVATIONS) {
    binary_or_numerical = "binary";
  }
  std::string filename = "./data/multifault/" + binary_or_numerical +
                         "_testing/" + m_behavior_str + "/" + m_behavior_str +
                         "_" + std::to_string(random_seed) + ".csv";
  return filename;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CEPuckFaultDetectionMultiFault,
                    "epuck_fault_detection_multifault")
