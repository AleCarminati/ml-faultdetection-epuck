#include "behavior.h"

CBehavior::CBehavior(Real maxWheelSpeed) : m_maxWheelSpeed(maxWheelSpeed) {}

CBehavior::~CBehavior() {}

Real CBehavior::GetMaxWheelSpeed() { return m_maxWheelSpeed; }

bool CBehavior::Action(CCI_EPuckProximitySensor::TReadings& tProxReads,
                       CCI_RangeAndBearingSensor::TReadings& tRABReads,
                       Real* wheelVelocity, std::string id_str) {
  return true;
}

/* The code of this functions is taken and adapted from Tarapore_PLOSONE_2017,
 * more precisely from the file behavior.cpp. */
void CBehavior::WheelSpeedFromHeadingVector(CVector2& m_cHeadingVector,
                                            Real* wheelVelocity) {
  /* Get the heading angle */
  CRadians cHeadingAngle = m_cHeadingVector.Angle().SignedNormalize();
  /* Get the length of the heading vector */
  Real fHeadingLength = m_cHeadingVector.Length();
  /* Clamp the speed so that it's not greater than MaxSpeed */
  Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_maxWheelSpeed);

  int TurningMechanism;

  /* Turning state switching conditions */
  if (cHeadingAngle.GetAbsoluteValue() <=
      CConfiguration::NO_TURN_THRESHOLD.GetValue()) {
    /* No Turn, heading angle very small */
    TurningMechanism = CConfiguration::NO_TURN;
  } else if (cHeadingAngle.GetAbsoluteValue() >
                 CConfiguration::NO_TURN_THRESHOLD.GetValue() &&
             cHeadingAngle.GetAbsoluteValue() <=
                 CConfiguration::SOFT_TURN_THRESHOLD.GetValue()) {
    /* Soft Turn, heading angle in between the two cases */
    TurningMechanism = CConfiguration::SOFT_TURN;
  } else if (cHeadingAngle.GetAbsoluteValue() >
             CConfiguration::SOFT_TURN_THRESHOLD.GetValue()) {
    /* Hard Turn, heading angle very large */
    TurningMechanism = CConfiguration::HARD_TURN;
  }

  /* Wheel speeds based on current turning state */
  Real fSpeed1, fSpeed2;
  if (TurningMechanism == CConfiguration::NO_TURN) {
    /* Just go straight */
    fSpeed1 = fBaseAngularWheelSpeed;
    fSpeed2 = fBaseAngularWheelSpeed;
  } else if (TurningMechanism == CConfiguration::SOFT_TURN) {
    /* Both wheels go straight, but one is faster than the other */
    Real fSpeedFactor = (CConfiguration::SOFT_TURN_THRESHOLD.GetValue() -
                         cHeadingAngle.GetAbsoluteValue()) /
                        CConfiguration::SOFT_TURN_THRESHOLD.GetValue();
    fSpeed1 =
        fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
    fSpeed2 =
        fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);

    /*
     * to make sure the speeds do not exceed the max speed
     *
     */
    if (fSpeed1 > m_maxWheelSpeed) fSpeed1 = m_maxWheelSpeed;

    if (fSpeed2 > m_maxWheelSpeed) fSpeed2 = m_maxWheelSpeed;

    if (fSpeed1 < -m_maxWheelSpeed) fSpeed1 = -m_maxWheelSpeed;

    if (fSpeed2 < -m_maxWheelSpeed) fSpeed2 = -m_maxWheelSpeed;
  } else if (TurningMechanism == CConfiguration::HARD_TURN) {
    /* Opposite wheel speeds */
    fSpeed1 = -m_maxWheelSpeed;
    fSpeed2 = m_maxWheelSpeed;
  }

  /* Apply the calculated speeds to the appropriate wheels */
  if (cHeadingAngle > CRadians::ZERO) {
    /* Turn Left */
    wheelVelocity[0] = fSpeed1;
    wheelVelocity[1] = fSpeed2;
  } else {
    /* Turn Right */
    wheelVelocity[0] = fSpeed2;
    wheelVelocity[1] = fSpeed1;
  }
}
