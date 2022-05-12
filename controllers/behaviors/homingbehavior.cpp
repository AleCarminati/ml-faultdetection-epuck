#include "homingbehavior.h"

CHomingBehavior::CHomingBehavior(Real maxWheelSpeed)
    : CBehavior(maxWheelSpeed) {}

bool CHomingBehavior::Action(CCI_EPuckProximitySensor::TReadings& tProxReads,
                             CCI_RangeAndBearingSensor::TReadings& tRABReads,
                             Real* wheelVelocity, std::string id_str) {
  CVector2 m_cHomingVector;
  CVector2 m_cHeadingVector;
  bool found = false;

  for (size_t i = 0; i < tRABReads.size() && !found; ++i) {
    CByteArray message = CByteArray(tRABReads[i].Data);

    if (message[CConfiguration::BEACON_BYTE_INDEX] ==
        CConfiguration::BEACON_MESSAGE) {
      found = true;
      m_cHomingVector =
          CVector2(tRABReads[i].Range, tRABReads[i].HorizontalBearing);
    }
  }

  // Beacon not found. The homing behavior can't be executed.
  if (!found) {
    return false;
  }

  m_cHeadingVector = GetMaxWheelSpeed() * m_cHomingVector.Normalize() +
                     GetMaxWheelSpeed() * CVector2(1.0f, 0.0f);

  WheelSpeedFromHeadingVector(m_cHeadingVector, wheelVelocity);

  return true;
}
