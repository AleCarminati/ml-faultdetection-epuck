#ifndef DISPERSEBEHAVIOR_H_
#define DISPERSEEBEHAVIOR_H_

#include "behavior.h"
#include <vector>
#include <algorithm>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Util functions. */
#include "../util/util.h"



using namespace argos;

class CDisperseBehavior : public CBehavior
{
public:
	CDisperseBehavior(Real maxWheelSpeed);

	virtual bool Action(CCI_EPuckProximitySensor::TReadings& tProxReads,
		CCI_RangeAndBearingSensor::TReadings& tRABReads, Real* wheelVelocity,
		std::string id_str);
};

#endif
