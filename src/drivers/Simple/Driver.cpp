#include "Driver.h"

namespace pomdp
{

Driver::~Driver() {
	if (agentScenario == "planner") {
        delete planner;
        delete simulator;
    }
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
    discount = 1.0;
    targetSpeedReached = false;
	elapsed = STEER_ACTION_FREQ; // Use planner to steer immediately after target speed is reached

    // Grid search for hyperparameters
	if (runs == 0 || runs == TARGET_RUNS) {
        runs = 0;
		totalReward = 0;
		bool isFinished = false;
		if (SEARCH_DISCOUNT) {
			std::tie(isFinished, agentScenario, discount, actions, binSize, planningTime, numSimulations) = GridSearch::getInstance().getNextDiscountScenario();
		} else {
			std::tie(isFinished, agentScenario, discount, actions, binSize, planningTime, numSimulations) = GridSearch::getInstance().getNextScenarios();
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
		planner = new PomcpPlanner<State, Observation, Action, pomcp::VectorBelief<State>>{ *simulator, 
			planningTime, numSimulations, RESAMPLING_TIME, THRESHOLD, EXPLORATION_CTE, PARTICLES, PARTICLE_REINV, (unsigned) (numSimulations * TRANSFER_QUOTA), (unsigned) (numSimulations * TRANSFER_QUOTA) }; 
	}
	// We use the #run as seed so that the episodes of the different solution methods are comparable
	driverModel = new DriverModel(driverActions, runs);

	// CSV Writer
	string fileName = agentScenario + " a" + std::to_string(GridSearch::getInstance().actionsScenarioIdx);
	if (SEARCH_DISCOUNT) {
		fileName += " d" + std::to_string(GridSearch::getInstance().discountScenarioIdx);
	} else if (agentScenario == "planner") {
		fileName += " b" + std::to_string(GridSearch::getInstance().binsScenarioIdx);
		fileName += " p" + std::to_string(GridSearch::getInstance().planningTimeScenarioIdx);
		fileName += " s" + std::to_string(GridSearch::getInstance().numSimsScenarioIdx);
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
	car->ctrl.gear = DrivingUtil::getGear(car);
	car->ctrl.brakeCmd = DrivingUtil::getBrake(car);
	if (car->ctrl.brakeCmd == 0.0) {
		car->ctrl.accelCmd = DrivingUtil::getAccel(car);
	} else {
		car->ctrl.accelCmd = 0.0;
	}

	// We want to reach a certain initial speed before planning starts
	if (!targetSpeedReached) {
		car->ctrl.steer = DrivingUtil::getOptimalSteer(car);
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
			Observation obs = Observation(*s, lastDriverAction, actionsCount, actions);

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

			double distToStart = car->_trkPos.seg->lgfromstart + 
				(car->_trkPos.seg->type == TR_STR ? car->_trkPos.toStart : car->_trkPos.toStart * car->_trkPos.seg->radius);
			double distToMiddle = 2*car->_trkPos.toMiddle/(car->_trkPos.seg->width);
			bool isDistracted = driverModel->getState().isDistracted;
			
			bool isTerminal = State::isTerminal(*s);

			writer << std::make_tuple(runs, actionsCount, cheat ? "cheat" : "fair", isTerminal, size, depth, speed, torcsState.angle, reward, rewardGain, distToStart, 
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

		// Determine driver's action (discretized)
		driverModel->update(torcsState);
		float driverAction = driverModel->getAction();
		
		unsigned agentActionIdx = 0;
		float agentAction;
		float optimalAction = DrivingUtil::getOptimalSteer(car);
		// float optimalAction = utils::Discretizer::discretize(driverActions, DrivingUtil::getOptimalSteer(car));
		if (agentScenario == "planner") {
			if (!cheat) {
				// POMCP planner
				agentActionIdx = planner->getAction();
			} else {
				// Select random
				agentActionIdx = actions[utils::RANDOM(actions.size())];
			}
			agentAction = actions[agentActionIdx];
		} else if (agentScenario == "optimal") {
			// Optimal response - We use elements of the hidden state of the environment
			tCar realCarState;
			ReInfo->_reSimItf.getState(&realCarState);
			// double maxReward = -100;
			double minDistance = 2;
			for (unsigned a = 0; a < actions.size(); a ++) {
				DriverModelState modelState;
				State state{ *s, realCarState, modelState, 0};
				State nextState;
				double reward;
				simulator->simulateStep(state, actions[a], driverAction, nextState, reward); // TODO: Look multiple steps ahead?
				tCarElt* car = nextState.situation.cars[0];
				double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
				// if (reward > maxReward) {
				// 	maxReward = reward;
				// 	agentActionIdx = a;
				// 	agentAction = actions[agentActionIdx];
				// }
				if (absDistToMiddle < minDistance) {
					minDistance = absDistToMiddle;
					agentActionIdx = a;
					agentAction = actions[agentActionIdx];
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
		car->_steerCmd *= 1 - elapsed / STEER_ACTION_FREQ; // TODO: Fix
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

	if (!targetSpeedReached && (int) (speed * 10 + .05) == (int) (TARGET_SPEED * 10 + .05)) {
		targetSpeedReached = true;
	}
}

}