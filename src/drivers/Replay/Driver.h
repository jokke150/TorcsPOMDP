#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <tuple>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "csv.hpp"

#include "Constants.h"
#include "DrivingUtil.h"

using std::vector;
using std::tuple;
using std::string;

using namespace csv;

namespace pomdp
{

class Driver {
	public:
		Driver() = default;
		~Driver();

		/* callback functions called from TORCS */
		void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
		void newRace(tCarElt* car, tSituation *s, tRmInfo *ReInfo);
		void drive(tSituation *s, tRmInfo *ReInfo);
		void endRace(tSituation *s);
		tCarElt *getCarPtr() { return car; }
		tTrack *getTrackPtr() { return track; }
		float getSpeed() { return speed; }
		void getInitialState(tCar& initState, tSituation& initSituation) { initState = this->initState; initSituation = this->initSituation; }

	private:
		/* utility functions */
		void update(tSituation *s);
		
		/* experiment state */
		tCar initState;
		tSituation initSituation;
		int performedActions;
		std::vector<float> actions;

		/* race state */
		float speed;
		tCarElt *car; // pointer to tCarElt struct
		bool targetSpeedReached;
		bool targetPosReached;
		double elapsed;

		/* track variables */
		tTrack* track;

};

}

#endif