#ifndef _TORCS_SIM_H_
#define _TORCS_SIM_H_

#include <vector>
#include <boost/functional/hash.hpp>

#include <raceman.h>
#include <car.h> 

#include "MonteCarloSimulator.hpp"
#include "Random.hpp"
#include "Pomcp.hpp"
#include "TorcsPomdp.hpp"
#include "DrivingUtil.h"

using namespace pomcp;

namespace pomdp
{

class Driver;

/**
 * class TorcsSimulator
 *
 * An implementation of Simulator<State, Observation, Action> for TORCS.
 * See MontecarloSimulator.hpp for more information about the interface of the simulator
 */
class TorcsSimulator : public Simulator<State, Observation, Action>
{
public:
	/**
	* @param raceEngineInfo: Information about race engine, e.g. the track. Needed for init of generative model.
	* @param initSituation: Reference to the initial torcs state
    * Create a new simulator
    */
	TorcsSimulator(tSituation& situation, tRmInfo& raceEngineInfo, vector<Action>& actions, vector<Action>& driverActions, int numBins, double discount);
	virtual ~TorcsSimulator();
	virtual double getDiscount() const {return discount;}
	virtual State& sampleInitialState(State& state) const;
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward,unsigned depth) const;
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward,unsigned depth) const;
	virtual unsigned getNumActions() const {return actions.size();}
	virtual const Action& getAction(unsigned actionIndex) const {return actions[actionIndex];}
	virtual bool allActionsAreValid(const State& state) const {return true;}
	virtual bool isValidAction(const State& state, unsigned actionIndex) const {return true;}

	void test(const State& origState) const; // TODO: Remove
private:
	tRmInfo& raceEngineInfo;
	tSituation& realSituation;
	vector<Action>& actions;
	vector<Action>& driverActions;
	int numBins;
	double discount;
	
	// Generative model
	tModList *modList = 0;
	tSimItf genModel;
	void loadGenModel();
};

inline
TorcsSimulator::TorcsSimulator(tSituation& situation, tRmInfo& raceEngineInfo, vector<Action>& actions, vector<Action>& driverActions, int numBins, double discount) 
	: raceEngineInfo{raceEngineInfo}, realSituation{situation}, actions{ actions }, driverActions{ driverActions }, numBins{ numBins }, discount { discount }
{
	loadGenModel();
}

inline
TorcsSimulator::~TorcsSimulator() {
	GfModUnloadList(&modList);
}

inline   
State& TorcsSimulator::sampleInitialState(State& state) const
{
	tCar envState;
	raceEngineInfo._reSimItf.getState(&envState); // WE CHEAT HERE BY USING THE REAL TORCS STATE!
	DriverModelState modelState = DriverModel::sampleState(driverActions);
	state = State{ realSituation, envState, modelState, 0 };
	return state;
}

inline
bool TorcsSimulator::simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward, unsigned depth) const
{
	// test(state);

	const Action& agentAction = getAction(actionIndex);
	
	nextState = state;
	tSituation* situation = &nextState.situation;

	tCarElt* car = situation->cars[0];

	// Get driver's action
	TorcsState torcsState{ *situation };
	DriverModel::updateInPlace(torcsState, nextState.modelState, driverActions);
	float driverAction = nextState.modelState.action;
	
	// Combine steering actions
	car ->_steerCmd = std::max(std::min(driverAction + agentAction, 1.0f), -1.0f);

	// Set simulator's internal car state
	genModel.setState(&nextState.car);
	
	// Simulate situation
	double elapsedTotal = 0;
	double elapsedLastCall = RCM_MAX_DT_ROBOTS;
	situation->deltaTime = RCM_MAX_DT_SIMU;
	do {
		if (elapsedLastCall >= RCM_MAX_DT_ROBOTS) {
			// Basic control updates
			car->ctrl.gear = DrivingUtil::getGear(car);
			car->ctrl.brakeCmd = DrivingUtil::getBrake(car);
			if (car->ctrl.brakeCmd == 0.0) {
				car->ctrl.accelCmd = DrivingUtil::getAccel(car);
			} else {
				car->ctrl.accelCmd = 0.0;
			}
			elapsedLastCall = 0;
		}
		genModel.update(situation, RCM_MAX_DT_SIMU, -1);
		situation->currentTime += RCM_MAX_DT_SIMU;
		elapsedTotal += RCM_MAX_DT_SIMU;
		elapsedLastCall += RCM_MAX_DT_SIMU;
	} while (elapsedTotal <= STEER_ACTION_FREQ);

	// Set reward after obtaining next state as it is based upon it
	reward = RewardCalculator::reward(*situation, agentAction);

	nextState.actionsCount++;
	observation = Observation{ *situation, driverAction, nextState.actionsCount, actions};
    return nextState.isTerminal();
}

// inline
// void TorcsSimulator::test(const State& origState) const {
// 	State nextState = origState;
// 	tSituation* situation = &nextState.situation;
// 	tCarElt* carElt = nextState.situation.cars[0];
// 	for (int i = 0; i < 10000; i++) {
// 		// Basic control updates
// 		carElt->ctrl.gear = DrivingUtil::getGear(carElt);
// 		carElt->ctrl.brakeCmd = DrivingUtil::getBrake(carElt);
// 		if (carElt->ctrl.brakeCmd == 0.0) {
// 			carElt->ctrl.accelCmd = DrivingUtil::getAccel(carElt);
// 		} else {
// 			carElt->ctrl.accelCmd = 0.0;
// 		}

// 		// Get driver's action
// 		TorcsState torcsState{ *situation };
// 		DriverModel::updateInPlace(torcsState, nextState.modelState, driverActions);
// 		float driverAction = nextState.modelState.action;
		
// 		// Combine steering actions
// 		car ->_steerCmd = std::max(std::min(driverAction + agentAction, 1.0f), -1.0f);

// 		realSituation.cars[0]->ctrl = carElt->ctrl;

// 		// Set simulator's internal car state
// 		genModel.setState(&nextState.car);

// 		double elapsed = 0;
// 		while (elapsed <= RCM_MAX_DT_ROBOTS + RCM_MAX_DT_SIMU) {
// 			genModel.update(situation, RCM_MAX_DT_SIMU, -1);
// 			situation->currentTime += RCM_MAX_DT_SIMU;
// 			elapsed += RCM_MAX_DT_SIMU;
// 		}

// 		elapsed = 0;
// 		while (elapsed <= RCM_MAX_DT_ROBOTS + RCM_MAX_DT_SIMU) {
// 			raceEngineInfo._reSimItf.update(&realSituation, RCM_MAX_DT_SIMU, -1);
// 			realSituation.currentTime += RCM_MAX_DT_SIMU;
// 			elapsed += RCM_MAX_DT_SIMU;
// 		}

// 		tCar realCar;
// 		raceEngineInfo._reSimItf.getState(&realCar); 

// 		Observation obs = Observation(*situation, 0, 1, actions);
// 		Observation obs1 = Observation(realSituation, 0, 1, actions);
	
// 		nextState = State{ nextState };
// 	}
// }

inline
bool TorcsSimulator::simulate(const State& state, unsigned actionIndex, State& nextState, double& reward,unsigned depth) const
{
	Observation observation;
	bool stop = simulate(state, actionIndex, nextState, observation, reward, depth);
	return stop;
}

inline
void TorcsSimulator::loadGenModel() {
    if (!modList) {
        const int BUFSIZE = 1024;
        char buf[BUFSIZE];

        const char* dllname = "simuv2";
        snprintf(buf, BUFSIZE, "%smodules/simu/copy/%s.%s", GetLibDir (), dllname, DLLEXT);
        if (GfModLoad(0, buf, &modList)) throw std::runtime_error("Could not load simu.");
        modList->modInfo->fctInit(modList->modInfo->index, &genModel);
        genModel.init(raceEngineInfo.s->_ncars, raceEngineInfo.track, raceEngineInfo.raceRules.fuelFactor, raceEngineInfo.raceRules.damageFactor);
    }
}

}

#endif