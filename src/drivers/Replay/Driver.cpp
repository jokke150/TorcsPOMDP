#include "Driver.h"

namespace pomdp
{

static const string ACTIONS_FILE = "/home/jokke/pCloudDrive/Utrecht/Thesis/Data/Preferred - 50 runs - 1000 actions - discount horizon 25 - exp const 1.5 - reduced/PREFERRED FULL NORM planner r50 a1000 s2500 c1.500000 d0.908518 reinv initial att prefer.csv";

Driver::~Driver() 
{
	CSVFormat format;
	format.delimiter(',').quote(false).header_row(0);
	CSVReader reader(ACTIONS_FILE, format);
	for (CSVRow& row: reader) { // Input iterator
		int run = row["Run"].get<int>();
		float action = row["Combined"].get<float>();
		if (run == 1) {
			actions.push_back(action);
		} else {
			break;
		}	
	}
}

/* Called for every track change or new race. */
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
	track = t;
}


/* Start a new race. */
void Driver::newRace(tCarElt* car, tSituation *s, tRmInfo *ReInfo)
{
	this->car = car;

	performedActions = 0;
	targetSpeedReached = false;
	targetPosReached = false;

	speed = car->pub.speed;
}


/* Drive during race. */
void Driver::drive(tSituation *s, tRmInfo *ReInfo)
{
	update(s);

	// Basic control updates
	car->ctrl.gear = DrivingUtil::getGear(*car);
	car->ctrl.brakeCmd = DrivingUtil::getBrake(*car);
	if (car->ctrl.brakeCmd == 0.0) {
		car->ctrl.accelCmd = DrivingUtil::getAccel(*car);
	} else {
		car->ctrl.accelCmd = 0.0;
	}

	// We want to reach a certain initial speed before planning starts
	if (!targetSpeedReached || !targetPosReached) {
		car->ctrl.steer = DrivingUtil::getOptimalSteer(*car);
		return;
	}

	if (elapsed >= STEER_ACTION_FREQ) {
		// Combine steering actions
		car->_steerCmd = actions[performedActions];
		performedActions++;
	} else {
		elapsed += RCM_MAX_DT_ROBOTS;
	}
}

/* End of the current race */
void Driver::endRace(tSituation *s) {}

void Driver::update(tSituation *s)
{
	speed = car->pub.speed;

	// We want to start with a certain speed
	if (!targetSpeedReached && (int) (speed * 10 + .05) == (int) (TARGET_SPEED * 10 + .05)) {
		targetSpeedReached = true;
	}

	// We want to start on a curve segment
	if (targetSpeedReached && !targetPosReached && car->pub.trkPos.seg->type != TR_STR) {
		targetPosReached = true;
	}
}

}