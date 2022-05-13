#ifndef FAULT_DETECTION_MULTIFAULT_H
#define FAULT_DETECTION_MULTIFAULT_H

#include "fault_detection.h"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEPuckFaultDetectionMultiFault : public CEPuckFaultDetection {
 public:
  CEPuckFaultDetectionMultiFault();

  /* Class destructor. */
  virtual ~CEPuckFaultDetectionMultiFault() {}

 private:
  virtual bool check_fault(int id, std::string fault) const override;

  virtual std::string generate_output_filename() const override;
};

#endif
