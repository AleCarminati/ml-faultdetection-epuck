#include "flockingbehavior.h"

CFlockingBehavior::CFlockingBehavior(Real maxWheelSpeed)
    : CBehavior(maxWheelSpeed), m_time_counter(0u) {
  /* Initialize all the elements of the accumulator matrix and of the
   * displacement arrayusing invalid objects.
   */
  DisplacementAccumulator accumulatorMatrix[N_ROBOTS][CONTROL_STEPS_WINDOW];
  memcpy(m_accumulatorMatrix, accumulatorMatrix,
         sizeof(DisplacementAccumulator) * N_ROBOTS * CONTROL_STEPS_WINDOW);
  for (int i = 0; i < N_ROBOTS; ++i) {
    for (int j = 0; j < CONTROL_STEPS_WINDOW; ++j) {
      m_accumulatorMatrix[i][j] = {CVector2(), CVector2(), CRadians(), false};
    }
  }
  CVector2 displacements[N_ROBOTS];
  memcpy(m_displacements, displacements, sizeof(CVector2) * N_ROBOTS);
  for (int i = 0; i < N_ROBOTS; ++i) {
    m_displacements[i] = (CConfiguration::INVALID_DISPLACEMENT_VECTOR);
  }
}

bool CFlockingBehavior::Action(CCI_EPuckProximitySensor::TReadings& tProxReads,
                               CCI_RangeAndBearingSensor::TReadings& tRABReads,
                               Real* wheelVelocity, std::string id_str) {
  CVector2 flockingVector = CVector2();
  unsigned robotsInRangeFlock = 0u;
  CVector2 aggregationVector = CVector2();
  unsigned robotsInRangeAggr = 0u;
  Real averageDistanceNeighbors = 0.0f;
  id_str.erase(0, 2);
  UInt8 id_int = std::stoi(id_str);

  /* Sum the velocities of the robots in the last time time interval.
   * Attention: in this case we use the displacements because the time interval
   * is equal to 1 second, so, numerically, the velocities and the displacements
   * are equal.
   */
  for (int i = 0; i < N_ROBOTS; ++i) {
    if (m_displacements[i] != (CConfiguration::INVALID_DISPLACEMENT_VECTOR) &&
        i != id_int) {
      flockingVector += m_displacements[i];
      robotsInRangeFlock += 1;
    }
  }

  if (robotsInRangeFlock > 0u) {
    flockingVector /= robotsInRangeFlock;
  } else {
    /* It wasn't possible to compute the speed of any other robots => the
     * flocking behavior is useless.
     */
    return false;
  }

  /* Find the distance of the neighbors. The flocking behavior has an
   * aggregation component, in this way the robots can organize themselves in a
   * compact formation.
   */
  for (int i = 0; i < tRABReads.size(); ++i) {
    aggregationVector +=
        CVector2(tRABReads[i].Range, tRABReads[i].HorizontalBearing);
    robotsInRangeAggr += 1;
    averageDistanceNeighbors += tRABReads[i].Range;
  }
  if (robotsInRangeAggr > 3) {
    aggregationVector /= robotsInRangeAggr;
    averageDistanceNeighbors /= robotsInRangeAggr;
  } else {
    aggregationVector = CVector2();
    averageDistanceNeighbors = 0.0f;
  }
  /* The maximum range of the RAB is 100cm. This operation applies range
   * normalization to the average distance of the neighbors.
   */
  averageDistanceNeighbors /= 100.0f;

  /* The heading vector is a weighted sum between aggregation behavior's and
   * flocking behavior's vectors. The weights are based on the neighbors'
   * distance: if they are near, the flocking behavior prevails because the
   * robots are already in a compact formation; if they are distant the priority
   * is to move near them.
   */
  CVector2 m_cHeadingVector =
      GetMaxWheelSpeed() *
      ((1.0f - averageDistanceNeighbors) * flockingVector.Normalize() +
       averageDistanceNeighbors * aggregationVector.Normalize());

  WheelSpeedFromHeadingVector(m_cHeadingVector, wheelVelocity);

  return true;
}

void CFlockingBehavior::UpdateDisplacements(
    CCI_RangeAndBearingSensor::TReadings& tRABReads,
    const CCI_DifferentialSteeringSensor::SReading& tDifReads,
    float* lastWheelVel, std::string id_str) {
  computeDisplacementsMatrix(tRABReads, tDifReads, lastWheelVel,
                             &m_accumulatorMatrix[0][0], m_displacements,
                             CONTROL_STEPS_WINDOW, m_time_counter, id_str);

  /* The time counter gets updated at the end of the function because in the
   * first control step the time counter is equal to 0, that is the first index
   * of the array.
   */
  m_time_counter++;
}
