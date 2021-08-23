#ifndef _DRIVING_UTIL_H_
#define _DRIVING_UTIL_H_

#include <raceman.h>
#include <robottools.h>

#include "Constants.h"


namespace pomdp
{

class DrivingUtil {

    public: 
        /* driving functions */
        static float getAccel(tCarElt* car);
        static float getBrake(tCarElt* car);
        static int getGear(tCarElt* car);
        static float getOptimalSteer(tCarElt* car);

    private:
        DrivingUtil() = delete;
};

/* Compute fitting acceleration */
inline float DrivingUtil::getAccel(tCarElt* car)
{
	if (TARGET_SPEED > car->_speed_x + FULL_ACCEL_MARGIN) {
		return 1.0;
	} else {
		float gr = car->_gearRatio[car->_gear + car->_gearOffset];
		float rm = car->_enginerpmRedLine;
		return TARGET_SPEED / car->_wheelRadius(REAR_RGT) * gr / rm;
	}
}


inline float DrivingUtil::getBrake(tCarElt* car)
{
	if (TARGET_SPEED < car->_speed_x) {
		return MIN(1.0f, (car->_speed_x - TARGET_SPEED) / (FULL_ACCEL_MARGIN));
	}
	return 0.0;
}

/* Compute gear */
inline int DrivingUtil::getGear(tCarElt* car)
{
	if (car->_gear <= 0) {
		return 1;
	}
	float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
	float omega = car->_enginerpmRedLine/gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < car->_speed_x) {
		return car->_gear + 1;
	} else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = car->_enginerpmRedLine/gr_down;
		if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) {
			return car->_gear - 1;
		}
	}
	return car->_gear;
}


/* compute steer value */
inline float DrivingUtil::getOptimalSteer(tCarElt* car)
{
	float trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
	float angle = trackangle - car->_yaw;
	NORM_PI_PI(angle);

	return angle / car->_steerLock;
}

}

#endif