#ifndef AGGREGATEBEHAVIOR_H_
#define AGGREGATEBEHAVIOR_H_

#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include "behavior.h"
/* Util functions. */
#include "../util/util.h"

using namespace argos;

class CAggregateBehavior : public CBehavior {
 public:
  CAggregateBehavior(Real maxWheelSpeed);

  virtual bool Action(CCI_EPuckProximitySensor::TReadings& tProxReads,
                      CCI_RangeAndBearingSensor::TReadings& tRABReads,
                      Real* wheelVelocity, std::string id_str);
};

#endif
