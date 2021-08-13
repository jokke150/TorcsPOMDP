#include "Driver.h"

namespace pomdp
{

Driver::~Driver() {
	if (gridSearch.agentScenario == "planner") {
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
void Driver::newRace(tCarElt* car, tSituation *s)
{
	this->car = car;
	actionsCount = 0;
    reward = 0;
    discount = 1.0;
    targetSpeedReached = false;
	elapsed = STEER_FREQ; // Use planner to steer immediately after target speed is reached

    // Grid search for hyper parameters
	if (runs == TARGET_RUNS) {
        runs = 0;
	}
	if (runs == 0) {
		bool isFinished = false;
		if (SEARCH_DISCOUNT) {
			std::tie(isFinished, discount, actions, binSize, planningTime) = gridSearch.getNextDiscountScenario();
		} else {
			std::tie(isFinished, discount, actions, binSize, planningTime) = gridSearch.getNextScenarios();
		}
		if (isFinished) {
			// Experiment is finished
			car->END = true;
			return;
		}
		// Driver can only steer half. Ultimately, the agent is in control.
        driverActions = gridSearch.agentScenario == "planner" ? std::vector<Action>(actions.begin() + 1, actions.end() - 1) : actions; 
		Observation::setAngleBins(binSize);
        Observation::setMiddleBins(binSize);
	}

	// CSV Writer
    ofs.open (gridSearch.agentScenario
		+ " a" + std::to_string(gridSearch.actionsScenarioIdx) 
		+ (gridSearch.agentScenario == "planner" ? " b" + std::to_string(gridSearch.binsScenarioIdx) : "") 
		+ (gridSearch.agentScenario == "planner" ? " p" + std::to_string(gridSearch.planningTimeScenarioIdx) : "") 
		+ (SEARCH_DISCOUNT? " d" + std::to_string(gridSearch.discountScenarioIdx) : "")
		+ ".csv"
		, std::ofstream::out | std::ofstream::app);

    if (runs == 0) {
        writer << std::vector<std::string>({"Run", "Count", "Cheat", "Terminal", "Size", "Depth", "Speed", "Angle", "Reward", "Gain", "From Start", 
                            "To Middle", "Distracted", "Time", "NumActions", "Optimal", "Combined", 
                            "Agent", "Driver" });
    }

    runs++;
}


/* Drive during race. */
void Driver::drive(tSituation *s, tRmInfo *ReInfo)
{

	float oldSteer = car->ctrl.steer;
	memset(&car->ctrl, 0, sizeof(tCarCtrl));

	update(s);

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

	if (elapsed >= STEER_FREQ) {
		elapsed = 0;

		TorcsState torcsState{ *s };

		if (!actionsCount) {
			if (gridSearch.agentScenario == "planner") {
				simulator = new TorcsSimulator{ *s, *ReInfo, actions, driverActions, binSize, discount };
				planner = new PomcpPlanner<State, Observation, Action, pomcp::VectorBelief<State>>{ *simulator, 
					planningTime, RESAMPLING_TIME, THRESHOLD, EXPLORATION_CTE, PARTICLES }; 
			}
			driverModel = new DriverModel(driverActions);  
		} else {
			// Only update planner after first action
			Observation obs = Observation(*s, lastDriverAction, actionsCount, actions);

			bool cheat = false;
			unsigned depth, size;
			depth = size = 0;
			if (gridSearch.agentScenario == "planner") {
				planner->computeInfo(size,depth);
				cheat = planner->moveTo(lastActIdx, obs); // TODO: Actually, this determines if the planner will cheat for the next action, not the last one
			}

			// Calculate and sum up reward
			// discount *= simulator->getDiscount();
			// reward  += discount * RewardCalculator::reward(*s, actions[lastActIdx]);
			double rewardGain = RewardCalculator::reward(*s, actions[lastActIdx]);
			reward += rewardGain;

			double distToStart = car->_trkPos.seg->lgfromstart + 
				(car->_trkPos.seg->type == TR_STR ? car->_trkPos.toStart : car->_trkPos.toStart * car->_trkPos.seg->radius);
			double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
			bool isDistracted = driverModel->getState().isDistracted;
			unsigned numActions = driverModel->getState().numActions;

			// std::cout<<"__________________________________"<<std::endl;
			// std::cout<<"Count: "<<actionsCount<<std::endl;
			// std::cout<<"Size: "<<size<<std::endl;
			// std::cout<<"Depth: "<<depth<<std::endl;
			// std::cout<<"Speed: "<<speed<<std::endl;
			// std::cout<<"Angle: "<<angle<<std::endl;
			// std::cout<<"Reward: "<<reward<<std::endl;
			// std::cout<<"From Start: "<<distToStart<<std::endl;
			// std::cout<<"To Middle: "<<absDistToMiddle<<std::endl;
			// std::cout<<"Driver distracted: "<<isDistracted<<std::endl;
			// std::cout<<"Current time: "<<s->currentTime<<std::endl;
			// std::cout<<"Driver state duration: "<<timeEpisodeEnd<<std::endl;
			// std::cout<<"Optimal: "<<lastOptimalAction<<std::endl;
			// std::cout<<"Combined: "<<lastCombinedAction<<std::endl;
			// std::cout<<"Agent: "<<actions[lastActIdx]<<std::endl;
			// std::cout<<"Driver: "<<lastDriverAction<<std::endl; 
			
			bool isTerminal = State::isTerminal(*s);

			writer << std::make_tuple(runs, actionsCount, cheat ? "cheat" : "fair", isTerminal, size, depth, speed, torcsState.angle, reward, rewardGain, distToStart, 
									absDistToMiddle, isDistracted, s->currentTime, numActions, lastOptimalAction, lastCombinedAction,
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

		float driverAction = 0;
		int agentActionIdx = 0;
		float optimalAction = DrivingUtil::getOptimalSteer(car);
		optimalAction = utils::Discretizer::discretize(actions, optimalAction);
		if (gridSearch.agentScenario != "optimal") {
			// Determine driver's action (discretized)
			driverModel->update(torcsState);
			driverAction = driverModel->getAction();

			float agentAction = 0;
			if (gridSearch.agentScenario == "planner") {
				agentActionIdx = planner->getAction();
				agentAction = actions[agentActionIdx];
			}

			// Combine steering actions
			car->_steerCmd = std::max(std::min(driverAction + agentAction, 1.0f), -1.0f);
		} else {
			car->_steerCmd = optimalAction;
		}
		
		actionsCount++;
		lastActIdx = agentActionIdx;
		lastOptimalAction = optimalAction;
		lastCombinedAction = car->_steerCmd;
		lastDriverAction = driverAction;
	} else {
		elapsed += RCM_MAX_DT_ROBOTS;
		car->ctrl.steer = oldSteer;
	}
}

inline void Driver::restart(tCarElt* car)
{
    totalReward += reward;
    double avgReward = totalReward / runs;
    std::cout<<"Restarting"<<std::endl;
    std::cout<<"Average reward after "<<runs<<" runs: "<<avgReward<<std::endl;

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