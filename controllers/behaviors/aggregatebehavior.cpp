#include "aggregatebehavior.h"

CAggregateBehavior::CAggregateBehavior(Real maxWheelSpeed)
    : CBehavior(maxWheelSpeed) {}

bool CAggregateBehavior::Action(CCI_EPuckProximitySensor::TReadings& tProxReads,
                                CCI_RangeAndBearingSensor::TReadings& tRABReads,
                                Real* wheelVelocity, std::string id_str) {
  CVector2 m_cAggregationVector = CVector2(0.0f, 0.0f);

  unsigned robotsinrange = 0;
  for (size_t i = 0; i < tRABReads.size(); ++i) {
    if (tRABReads[i].Range < CConfiguration::RANGE_THRESHOLD_AGGREGATION) {
      m_cAggregationVector +=
          CVector2(tRABReads[i].Range, tRABReads[i].HorizontalBearing);
      robotsinrange++;
    }
  }

  if (robotsinrange <= 3u) {  // use > 3 instead of > 0 to avoid small
                              // aggregates of just 2 robots or just 3 robots.
    return false;
  }

  m_cAggregationVector /= robotsinrange;

  GetMaxWheelSpeed();
  m_cAggregationVector.Normalize();
  CSourceOfRandomness::m_pcRNG->Uniform(
      CRange<CRadians>(-CRadians::PI, CRadians::PI));

  CVector2 m_cHeadingVector =
      GetMaxWheelSpeed() *
      (1.0f * m_cAggregationVector.Normalize() +
       1.0f *
           CVector2(1.0f, CSourceOfRandomness::m_pcRNG->Uniform(
                              CRange<CRadians>(-CRadians::PI, CRadians::PI))));

  WheelSpeedFromHeadingVector(m_cHeadingVector, wheelVelocity);

  return true;
}
