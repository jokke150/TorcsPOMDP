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
        static float getAccel(tCarElt* car);
        static float getBrake(tCarElt* car);
        static int getGear(tCarElt* car);
        static float getOptimalSteer(tCarElt* car);
		// static float getAllowedSpeed(tTrackSeg *segment, float CA);

    private:
        DrivingUtil() = delete;

		static v2d getTargetPoint(tCarElt* car);
		static float getDistToSegEnd(tCarElt* car);

		static constexpr float LOOKAHEAD_CONST = 17.0;	/* [m] */
		static constexpr float LOOKAHEAD_FACTOR = 0.33;	/* [-] */
};

/* Compute fitting acceleration */
inline 
float DrivingUtil::getAccel(tCarElt* car)
{
	if (TARGET_SPEED > car->_speed_x + FULL_ACCEL_MARGIN) {
		return 1.0;
	} else {
		float gr = car->_gearRatio[car->_gear + car->_gearOffset];
		float rm = car->_enginerpmRedLine;
		return TARGET_SPEED / car->_wheelRadius(REAR_RGT) * gr / rm;
	}
}


inline 
float DrivingUtil::getBrake(tCarElt* car)
{
	if (TARGET_SPEED < car->_speed_x) {
		return MIN(1.0f, (car->_speed_x - TARGET_SPEED) / (FULL_ACCEL_MARGIN));
	}
	return 0.0;
}

/* Compute gear */
inline 
int DrivingUtil::getGear(tCarElt* car)
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
inline 
float DrivingUtil::getOptimalSteer(tCarElt* car)
{
	// float trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
	// float angle = trackangle - car->_yaw;
	// NORM_PI_PI(angle);

	float targetAngle;
	v2d target = getTargetPoint(car);

	targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
	targetAngle -= car->_yaw;
	NORM_PI_PI(targetAngle);
	targetAngle -= car->_trkPos.toMiddle / car->_trkPos.seg->width;
	return (targetAngle / car->_steerLock); // TODO: /2 works well for some reason

	// float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    // NORM_PI_PI(angle); // normalize the angle between -PI and + PI
    // angle -= car->_trkPos.toMiddle / car->_trkPos.seg->width;

	// return angle / car->_steerLock / 2;
}

/* compute target point for steering */
inline
v2d DrivingUtil::getTargetPoint(tCarElt* car)
{
	tTrackSeg* seg = car->_trkPos.seg;
	// float lookahead = car->_speed_x * STEER_ACTION_FREQ;
	float lookahead = LOOKAHEAD_CONST + car->_speed_x * LOOKAHEAD_FACTOR;
	float length = getDistToSegEnd(car);

	while (length < lookahead) {
		seg = seg->next;
		length += seg->length;
	}

	length = lookahead - length + seg->length;
	float fromstart = seg->lgfromstart;
	fromstart += length;

	v2d s;
	s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0;
	s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0;

	if ( seg->type == TR_STR) {
		v2d d, n;
		n.x = (seg->vertex[TR_EL].x - seg->vertex[TR_ER].x)/seg->length;
		n.y = (seg->vertex[TR_EL].y - seg->vertex[TR_ER].y)/seg->length;
		n.normalize();
		d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;
		d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;
		return s + d*length;
	} else {
		v2d c, n;
		c.x = seg->center.x;
		c.y = seg->center.y;
		float arc = length/seg->radius;
		float arcsign = (seg->type == TR_RGT) ? -1.0 : 1.0;
		arc = arc*arcsign;
		s = s.rotate(c, arc);
		n = c - s;
		n.normalize();
		return s;
	}
}

/* Compute the length to the end of the segment */
inline
float DrivingUtil::getDistToSegEnd(tCarElt* car)
{
	if (car->_trkPos.seg->type == TR_STR) {
		return car->_trkPos.seg->length - car->_trkPos.toStart;
	} else {
		return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
	}
}


/* Compute the allowed speed on a segment */
// inline
// float DrivingUtil::getAllowedSpeed(tTrackSeg *segment, float CA)
// {
// 	if (segment->type == TR_STR) {
// 		return FLT_MAX;
// 	} else {
// 		float arc = 0.0;
// 		tTrackSeg *s = segment;

// 		while (s->type == segment->type && arc < PI/2.0) {
// 			arc += s->arc;
// 			s = s->next;
// 		}

// 		arc /= PI/2.0;
// 		float mu = segment->surface->kFriction;
// 		float r = (segment->radius + segment->width/2.0)/sqrt(arc);
// 		return sqrt((mu*G*r)/(1.0 - MIN(1.0, r*CA*mu/mass)));
// 	}
// }

}

#endif