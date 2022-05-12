#ifndef FLOCKINGBEHAVIOR_H_
#define FLOCKINGBEHAVIOR_H_

#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include "behavior.h"
/* Util functions. */
#include "../util/util.h"

/* The number of control steps that are in the window of time that we consider
 * to compute the average speed of a neighbor. Value taken from
 * Tarapore_PLOSONE_2017, more precisely from epuck_hom_swarm.cpp. In their
 * work, they use a time window of 1 second, which corresponds to 10 control
 * steps of the controller.
 */
#define CONTROL_STEPS_WINDOW 10

using namespace argos;

class CFlockingBehavior : public CBehavior {
 public:
  CFlockingBehavior(Real maxWheelSpeed);

  virtual bool Action(CCI_EPuckProximitySensor::TReadings& tProxReads,
                      CCI_RangeAndBearingSensor::TReadings& tRABReads,
                      Real* wheelVelocity, std::string id_str);

  /* This function updates the DisplacementAccumulators of the robot. It must be
   * called at every control step independently from the behavior that will be
   * followed in that control step. For this reason, this function is
   * independent of the function Action, which gets called only if this behavior
   * is selected. The parameter lastWheelVel must be an array of size 2 that
   * cointains the speed of the left wheel (in first position) and the speed of
   * the right wheel (in second position) of the observer robot in the last
   * control step.
   */
  virtual void UpdateDisplacements(
      CCI_RangeAndBearingSensor::TReadings& tRABReads,
      const CCI_DifferentialSteeringSensor::SReading& tDifReads,
      float* lastWheelVel, std::string id_str);

  /* A matrix of DisplacementAccumulator. It contains N_ROBOT rows: one for each
   * node in the swarm. The row with the same index as the observer's ID won't
   * be used, thus the validity of its elements will always be false.
   */
  DisplacementAccumulator m_accumulatorMatrix[N_ROBOTS][CONTROL_STEPS_WINDOW];

  /* When UpdateDisplacements is called, the displacements of each robot in the
   * last n steps is computed. This array saves these values, to be used by the
   * Action function.
   */
  CVector2 m_displacements[N_ROBOTS];

  /* This variable is a counter of the control steps. It is necessary to find
   * which element of the accumulator matrix is old enough to be used to compute
   * the displacement.
   */
  unsigned int m_time_counter;
};

#endif
