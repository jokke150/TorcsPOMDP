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

#include <csv.hpp>

#include "GridSearch.h"
#include "TorcsSimulator.hpp"
#include "Pomcp.hpp"
#include "Constants.h"
#include "DrivingUtil.h"

using std::vector;
using std::tuple;

using namespace pomcp;
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
		void restart(tCarElt* car);

		/* experiment state */
		unsigned int runs = 0;
		bool setInitialState = false;
		tCar initState;
		tSituation initSituation;
		double totalReward;
		
		/* episode state */
		unsigned long actionsCount;
		TorcsSimulator* simulator;
		PomcpPlanner<State, Observation, Action, pomcp::VectorBelief<State>>* planner;
		DriverModel* driverModel;
		double reward;
		bool cheat;

		/* grid search state */
		std::string agentScenario;
		vector<float> actions;
		vector<float> driverActions;
		int binSize;
		unsigned numSimulations;
		double discount;
		double exp_const;

		/* last actions */
		unsigned int lastActIdx;
		float lastDriverAction;
		float lastOptimalAction;
		float lastCombinedAction;

		/* persistence */
		std::ofstream ofs;
		csv::CSVWriter<std::ofstream, false> writer = make_csv_writer_buffered(ofs);

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