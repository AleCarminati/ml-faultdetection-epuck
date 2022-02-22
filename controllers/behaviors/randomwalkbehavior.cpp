#include "randomwalkbehavior.h"

CRandomWalkBehavior::CRandomWalkBehavior(Real maxWheelSpeed):
	CBehavior(maxWheelSpeed){}

bool CRandomWalkBehavior::Action(CCI_EPuckProximitySensor::TReadings& tProxReads,
		CCI_RangeAndBearingSensor::TReadings& tRABReads, Real* wheelVelocity,
        std::string id_str){

    if (CConfiguration::CHANGE_DIRECTION_PROBABILITY_RANDOMWALK >= CSourceOfRandomness::m_pcRNG->Uniform(CRange<Real>(0.0, 1.0))){
        Real fSpeed;

        CRadians angle = CSourceOfRandomness::m_pcRNG->Uniform(CRange<CRadians>(-CRadians::PI, CRadians::PI));

        fSpeed = angle.GetAbsoluteValue() * CConfiguration::HALF_INTERWHEEL_DISTANCE;
        fSpeed = Min<Real>(fSpeed, GetMaxWheelSpeed());
        
        if(angle.GetValue() > 0.0f){
            // Turn right
            wheelVelocity[0]  = fSpeed;
            wheelVelocity[1] = -fSpeed;
        }else{
            // Turn left
            wheelVelocity[0] = -fSpeed;
            wheelVelocity[1] =  fSpeed;
        }

    }
    else{
        // Go straight
        wheelVelocity[0] = GetMaxWheelSpeed();
        wheelVelocity[1] = GetMaxWheelSpeed();
    }

    return true;
}