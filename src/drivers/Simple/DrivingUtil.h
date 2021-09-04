#ifndef _DRIVING_UTIL_H_
#define _DRIVING_UTIL_H_

#include <raceman.h>
#include <robottools.h>

#include "linalg.h"

#include "Constants.h"


namespace pomdp
{

class DrivingUtil {

    public: 
        /* driving functions */
        static float getAccel(const tCarElt& car);
        static float getBrake(const tCarElt& car);
        static int getGear(const tCarElt& car);
        static float getOptimalSteer(const tCarElt& car);
		static float getAngle(const tCarElt& car);
		static double getDistToStart(const tCarElt& car);
		static double getDistToMiddle(const tCarElt& car);

    private:
        DrivingUtil() = delete;

		static v2d getTargetPoint(const tCarElt& car);
		static float getDistToSegEnd(const tCarElt& car);
};

/* Compute fitting acceleration */
inline 
float DrivingUtil::getAccel(const tCarElt& car)
{
	if (TARGET_SPEED > car._speed_x + FULL_ACCEL_MARGIN) {
		return 1.0;
	} else {
		float gr = car._gearRatio[car._gear + car._gearOffset];
		float rm = car._enginerpmRedLine;
		return TARGET_SPEED / car._wheelRadius(REAR_RGT) * gr / rm;
	}
}


inline 
float DrivingUtil::getBrake(const tCarElt& car)
{
	if (TARGET_SPEED < car._speed_x) {
		return MIN(1.0f, (car._speed_x - TARGET_SPEED) / (FULL_ACCEL_MARGIN));
	}
	return 0.0;
}

/* Compute gear */
inline 
int DrivingUtil::getGear(const tCarElt& car)
{
	if (car._gear <= 0) {
		return 1;
	}
	float gr_up = car._gearRatio[car._gear + car._gearOffset];
	float omega = car._enginerpmRedLine/gr_up;
	float wr = car._wheelRadius(2);

	if (omega*wr*SHIFT < car._speed_x) {
		return car._gear + 1;
	} else {
		float gr_down = car._gearRatio[car._gear + car._gearOffset - 1];
		omega = car._enginerpmRedLine/gr_down;
		if (car._gear > 1 && omega*wr*SHIFT > car._speed_x + SHIFT_MARGIN) {
			return car._gear - 1;
		}
	}
	return car._gear;
}


/* compute steer value */
inline 
float DrivingUtil::getOptimalSteer(const tCarElt& car)
{
	float targetAngle;
	v2d target = getTargetPoint(car);
	targetAngle = atan2(target.y - car._pos_Y, target.x - car._pos_X);
	targetAngle -= car._yaw;
	NORM_PI_PI(targetAngle);
	return (targetAngle / car._steerLock);
}

/* compute target point for steering */
inline
v2d DrivingUtil::getTargetPoint(const tCarElt& car)
{
	tTrackSeg *seg = car._trkPos.seg;
	float lookahead = STEER_LOOKAHEAD;
	float length = getDistToSegEnd(car);

	while (length < lookahead) {
		seg = seg->next;
		length += seg->length;	
	}
	
	length = lookahead - length + seg->length;
	v2d s;
	s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0;
	s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0;
	
	if ( seg->type == TR_STR) {
		v2d d;
		d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;
		d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;
		return s + d*length;
	} else {
		v2d c;
		c.x = seg->center.x;
		c.y = seg->center.y;
		float arc = length/seg->radius;
		float arcsign = (seg->type == TR_RGT) ? -1 : 1;
		arc = arc*arcsign;
		return s.rotate(c, arc);
	}
}

/* Compute the length to the end of the segment */
inline
float DrivingUtil::getDistToSegEnd(const tCarElt& car)
{
	if (car._trkPos.seg->type == TR_STR) {
		return car._trkPos.seg->length - car._trkPos.toStart;
	} else {
		return (car._trkPos.seg->arc - car._trkPos.toStart)*car._trkPos.seg->radius;
	}
}

inline 
float DrivingUtil::getAngle(const tCarElt& car)
{
	float trackangle = RtTrackSideTgAngleL(const_cast<tTrkLocPos*>(&(car._trkPos)));
	float angle = trackangle - car._yaw;
	NORM_PI_PI(angle); // normalize the angle between -PI and + PI
	return angle;
}


inline 
double DrivingUtil::getDistToStart(const tCarElt& car)
{
	return car._trkPos.seg->lgfromstart + 
				(car._trkPos.seg->type == TR_STR ? car._trkPos.toStart : car._trkPos.toStart * car._trkPos.seg->radius);
}

inline 
double DrivingUtil::getDistToMiddle(const tCarElt& car)
{
	if (TWO_LANE_TRACK) {
		// We divide by two here since the road is a two-lane road. 
		// The car will drive in the middle of the two lanes because that is easier to implement in TORCS. 
		// It does not make a difference for the performance evaluation.
		return (2 * car._trkPos.toMiddle) / (car._trkPos.seg->width / 2);
	} else {
		return 2 * car._trkPos.toMiddle / car._trkPos.seg->width; // This is for a one-lane track
	}
}

}

#endif