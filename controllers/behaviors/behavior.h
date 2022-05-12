#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Util functions. */
#include "../util/util.h"

/*Definition of constants.*/

using namespace argos;

class CBehavior {
 public:
  CBehavior(Real maxWheelSpeed);
  virtual ~CBehavior();

  /* Getter of the parameter m_maxWheelSpeed. */
  virtual Real GetMaxWheelSpeed();

  /* This function computes how the robot must move, based on data retrieved
   * from sensors. Based on the implementation, the function could decide to not
   * modify the wheels' speed: in this case it will return false. In all other
   * cases, it will return true.
   */
  virtual bool Action(CCI_EPuckProximitySensor::TReadings& tProxReads,
                      CCI_RangeAndBearingSensor::TReadings& tRABReads,
                      Real* wheelVelocity, std::string id_str);

  /* This function computes the speed of the wheels starting from the heading
   * vector.
   */
  virtual void WheelSpeedFromHeadingVector(CVector2& m_cHeadingVector,
                                           Real* wheelVelocity);

 private:
  /* Contains the max speed of the wheels, useful in computing how the robot
   * must move. */
  Real m_maxWheelSpeed;
};

#endif
