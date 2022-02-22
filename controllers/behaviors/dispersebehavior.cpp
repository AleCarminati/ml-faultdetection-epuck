#include "dispersebehavior.h"

CDisperseBehavior::CDisperseBehavior(Real maxWheelSpeed):
	CBehavior(maxWheelSpeed){}

bool CDisperseBehavior::Action(CCI_EPuckProximitySensor::TReadings& tProxReads,
	CCI_RangeAndBearingSensor::TReadings& tRABReads, Real* wheelVelocity,
  std::string id_str){

  CVector2 m_cDiffusionVector;

  m_cDiffusionVector.Set(0.0f, 0.0f);
  for(size_t i = 0; i <  tProxReads.size(); ++i)
  {
      m_cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
  }
  m_cDiffusionVector /= tProxReads.size();


  /* If the angle of the vector is small enough and the closest obstacle
    is far enough, ignore the vector and go straight, otherwise return
    it */
  if(m_cDiffusionVector.Angle().GetAbsoluteValue() < 
    CConfiguration::GO_STRAIGHT_ANGLE_THRESHOLD_DISPERSION.GetValue() && 
    m_cDiffusionVector.Length() < 
    CConfiguration::PROXIMITY_SENSOR_THRESHOLD_DISPERSION)
  {
      return false;
  }else{
    if(m_cDiffusionVector.Length() < 0.05) /* because of noise, we can have very small non-zero sensor readings. but we don't want to respond to them*/
      return false;
  }
  CVector2 m_cHeadingVector= -m_cDiffusionVector.Normalize()*GetMaxWheelSpeed();

  WheelSpeedFromHeadingVector(m_cHeadingVector, wheelVelocity);

  return true;
} 
