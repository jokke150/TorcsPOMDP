#include "Driver.h"

namespace pomdp
{

Driver::~Driver() {
	if (agentScenario == "planner") {
        delete planner;
    }
	delete simulator;
    delete driverModel;
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

	actionsCount = 0;
    reward = 0;
	cheat = false;
    targetSpeedReached = false;
	targetPosReached = false;
	elapsed = STEER_ACTION_FREQ; // Use planner to steer immediately after target speed is reached

    // Grid search for hyperparameters
	if (runs == 0 || runs == TARGET_RUNS) {
        runs = 0;
		totalReward = 0;
		bool isFinished = false;
		if (SEARCH_DISCOUNT) {
			std::tie(isFinished, agentScenario, discount, actions, binSize, numSimulations, exp_const) = GridSearch::getInstance().getNextDiscountScenario();
		} else {
			std::tie(isFinished, agentScenario, discount, actions, binSize, numSimulations, exp_const) = GridSearch::getInstance().getNextScenarios();
		}
		if (isFinished) {
			// Experiment is finished
			car->END = true;
			return;
		}
        driverActions = std::vector<Action>(actions.begin() + 1, actions.end() - 1); // Agent can overrule driver with "oversteering"
		Observation::setAngleBins(binSize);
        Observation::setMiddleBins(binSize);
	}

	simulator = new TorcsSimulator{ *s, *ReInfo, actions, driverActions, binSize, discount };
	if (agentScenario == "planner") {
		planner = new PomcpPlanner<State, Observation, Action, pomcp::VectorBelief<State>>
		{ 
			*simulator, 
			numSimulations, 
			THRESHOLD, 
			exp_const, PARTICLES, 
			PARTICLE_RESAMP, 
			(unsigned) (numSimulations * RESAMP_QUOTA), 
			(unsigned) (numSimulations * RESAMP_QUOTA), 
			PARTICLE_REINV, 
			(unsigned) (numSimulations * TRANSFER_QUOTA), 
			(unsigned) (numSimulations * TRANSFER_QUOTA) 
		}; 
	}
	// We use the #run as seed so that the episodes of the different solution methods are comparable
	driverModel = new DriverModel(driverActions, runs);

	// CSV Writer
	string fileName = agentScenario + " a" + std::to_string(GridSearch::getInstance().actionsScenarioIdx);
	if (SEARCH_DISCOUNT) {
		fileName += " d" + std::to_string(DISCOUNT_SCENARIOS[GridSearch::getInstance().discountScenarioIdx]);
	} else if (agentScenario == "planner") {
		fileName += " b" + std::to_string(GridSearch::getInstance().binsScenarioIdx);
		fileName += " s" + std::to_string(NUM_SIMS_SCENARIOS[GridSearch::getInstance().numSimsScenarioIdx]);
		fileName += " c" + std::to_string(EXP_CONST_SCENARIOS[GridSearch::getInstance().expConstScenarioIdx]);
		fileName += DRIVER_OVER_CORRECT ? " over correct" : "";
		fileName += !DRIVER_DISCRETE_ACTIONS ? " cont" : "";
		fileName += DRIVER_ACTION_NOISE ? " noisy" : "";
		fileName += AUTONOMOUS ? " autonomous" : "";
		fileName += PARTICLE_REINV ? " reinv " + std::to_string(TRANSFER_QUOTA) : "";
		fileName += PARTICLE_RESAMP ? " resamp" : "";
		fileName += " (1)";
	}
    ofs.open ( fileName + ".csv", std::ofstream::out | std::ofstream::app);

    if (runs == 0) {
        writer << std::vector<std::string>({"Run", "Count", "Cheat", "Terminal", "Size", "Depth", "Speed", "Angle", "Reward", "Gain", "From Start", 
                            "To Middle", "Distracted", "Time", "Actions Remaining", "Optimal", "Combined", 
                            "Agent", "Driver" });
    }

    runs++;

	// Console output
	std::cout << fileName + " - run " + std::to_string(runs) << std::endl;
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

	// Store initial state with correct speed for later episodes
	if (!setInitialState) {
		ReInfo->_reSimItf.getState(&initState);
		initSituation = *s;
		setInitialState = true;
	}

	if (elapsed >= STEER_ACTION_FREQ) {
		elapsed = 0;

		TorcsState torcsState{ *s };

		if (actionsCount) {
			// Only update planner after first action
			Observation obs = Observation(*s, lastDriverAction, actionsCount, driverActions);

			unsigned depth, size;
			depth = size = 0;
			if (agentScenario == "planner" && !cheat) {
				planner->computeInfo(size, depth);
				cheat = planner->moveTo(lastActIdx, obs); // TODO: Actually, this determines if the planner will cheat for the next action, not the last one
			}

			// Calculate and sum up reward
			// discount *= simulator->getDiscount();
			// reward  += discount * RewardCalculator::reward(*s, actions[lastActIdx]);
			double rewardGain = RewardCalculator::reward(*s, actions[lastActIdx]);
			reward += rewardGain;

			float angle = DrivingUtil::getAngle(*car);
			double distToStart = DrivingUtil::getDistToStart(*car);
			double distToMiddle = DrivingUtil::getDistToMiddle(*car);
			bool isDistracted = driverModel->getState().isDistracted;
			
			bool isTerminal = State::isTerminal(*s);

			writer << std::make_tuple(runs, actionsCount, cheat ? "cheat" : "fair", isTerminal, size, depth, speed, angle, reward, rewardGain, distToStart, 
									distToMiddle, isDistracted, s->currentTime, driverModel->getState().numActionsRemaining, lastOptimalAction, lastCombinedAction,
									actions[lastActIdx], lastDriverAction);

			// Restart race and start next run if terminal state is reached
			if (isTerminal) {
				std::cout<<"Terminal state reached after "<<actionsCount<<" actions."<<std::endl;
				return restart(car);
			}

			// Restart race and start next run if target number of actions is reached
			if (actionsCount == TARGET_ACTIONS) {
				std::cout<<"Episode finished after "<<actionsCount<<" actions."<<std::endl;
				return restart(car);
			}
		}

		// Determine driver's action
		float driverAction;
		if (AUTONOMOUS) {
			driverAction = 0;
		} else {
			driverModel->update(torcsState);
			driverAction = driverModel->getAction();
		}
		
		
		unsigned agentActionIdx = 0;
		float agentAction;
		float optimalAction = DrivingUtil::getOptimalSteer(*car);
		// optimalAction = utils::Discretizer::discretize(actions, optimalAction);
		if (agentScenario == "planner") {
			if (!cheat) {
				// POMCP planner
				agentActionIdx = planner->getAction();
			} else {
				// Select random
				agentActionIdx = utils::RANDOM(actions.size());
			}
			agentAction = actions[agentActionIdx];
		} else if (agentScenario == "optimal") {
			float minDistance = 10;
			for(int i = 0; i < actions.size(); i++) {
				float action = actions[i];
				float combined = std::max(std::min(driverAction + action, 1.0f), -1.0f);
				float optimal = DrivingUtil::getOptimalSteer(*car);
				float distance = abs(optimal - combined);
				if (distance < minDistance) {
					minDistance = distance;
					agentAction = action;
					agentActionIdx = i;
				}
			}
			// driverAction = 0;
			// agentAction = optimalAction;
		} else {
			// Driver only
			agentAction = 0;
		}

		// Combine steering actions
		car->_steerCmd = std::max(std::min(driverAction + agentAction, 1.0f), -1.0f);
		
		actionsCount++;
		lastActIdx = agentActionIdx;
		lastOptimalAction = optimalAction;
		lastCombinedAction = car->_steerCmd;
		lastDriverAction = driverAction;
	} else {
		elapsed += RCM_MAX_DT_ROBOTS;
	}
}

void Driver::restart(tCarElt* car)
{
    totalReward += reward;
    double avgReward = totalReward / runs;
    std::cout << "Restarting" << std::endl;
    std::cout << "Average reward after " << runs << " runs: " << avgReward << std::endl;

    writer.flush();
    ofs.close();

	if (agentScenario == "planner") {
        delete planner;
    }
	delete simulator;
    delete driverModel;

    car->RESTART = true;
}

/* End of the current race */
void Driver::endRace(tSituation *s) {}


/***************************************************************************
 *
 * utility functions
 *
***************************************************************************/

/* Update my private data every timestep */
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