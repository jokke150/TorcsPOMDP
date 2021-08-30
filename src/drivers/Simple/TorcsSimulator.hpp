#ifndef _TORCS_SIM_H_
#define _TORCS_SIM_H_

#include <vector>
#include <boost/functional/hash.hpp>

#include <raceman.h>
#include <car.h> 

#include "MonteCarloSimulator.hpp"
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
	virtual bool simulateStep(const State& state, const Action& agentAction, const Action& driverAction, State& nextState, double& reward) const;
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward,unsigned depth) const;
	virtual unsigned getNumActions() const {return actions.size();}
	virtual const Action& getAction(unsigned actionIndex) const {return actions[actionIndex];}
	virtual bool allActionsAreValid(const State& state) const {return true;}
	virtual bool isValidAction(const State& state, unsigned actionIndex) const {return true;}
	virtual bool transform(const State& state, const Observation& observation, State& transformedState) const;

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
	DriverModelState modelState = DriverModel::sampleState(driverActions, RANDOM, true);
	state = State{ realSituation, envState, modelState, 0 };
	return state;
}

inline
bool TorcsSimulator::simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward, unsigned depth) const
{
	// test(state);

	nextState = state;

	const Action& agentAction = getAction(actionIndex);

	// Get driver's action
	TorcsState torcsState{ nextState.situation };
	DriverModel::updateInPlace(torcsState, nextState.modelState, driverActions, RANDOM);
	float driverAction = nextState.modelState.action;

	bool isTerminal = simulateStep(state, agentAction, driverAction, nextState, reward);
	
	observation = Observation{ nextState.situation, driverAction, nextState.actionsCount, actions};
	
    return isTerminal;
}

inline
bool TorcsSimulator::simulateStep(const State& state, const Action& agentAction, const Action& driverAction, State& nextState, double& reward) const
{
	tSituation* situation = &nextState.situation;
	tCarElt* car = situation->cars[0];
	
	// Combine steering actions
	car ->_steerCmd = std::max(std::min(driverAction + agentAction, 1.0f), -1.0f);

	// Set simulator's internal car state
	genModel.setStatePointer(&nextState.car);
	
	// Simulate situation
	double elapsedTotal = 0;
	situation->deltaTime = RCM_MAX_DT_SIMU;
	while (elapsedTotal <= STEER_ACTION_FREQ) {
		// Basic control updates
		car->ctrl.gear = DrivingUtil::getGear(*car);
		car->ctrl.brakeCmd = DrivingUtil::getBrake(*car);
		if (car->ctrl.brakeCmd == 0.0) {
			car->ctrl.accelCmd = DrivingUtil::getAccel(*car);
		} else {
			car->ctrl.accelCmd = 0.0;
		}

		double elapsed = 0;
		while (elapsed <= RCM_MAX_DT_ROBOTS + RCM_MAX_DT_SIMU) {
			genModel.update(situation, RCM_MAX_DT_SIMU, -1);
			situation->currentTime += RCM_MAX_DT_SIMU;
			elapsed += RCM_MAX_DT_SIMU;
		}

		elapsedTotal += RCM_MAX_DT_ROBOTS;
	}

	// Set reward after obtaining next state as it is based upon it
	reward = RewardCalculator::reward(*situation, agentAction);

	nextState.actionsCount++;

	return nextState.isTerminal();;
}

inline
void TorcsSimulator::test(const State& origState) const {
	State nextState = origState;
	tSituation* situation = &nextState.situation;
	tCarElt* car = nextState.situation.cars[0];
	tCarElt* realCar = realSituation.cars[0];
	for (int i = 0; i < 10000; i++) {
		// Constant steering action
		car->_steerCmd = -0.1;
		realCar->ctrl = car->ctrl;

		// Set simulator's internal car state
		genModel.setStatePointer(&nextState.car);

		// Simulate situation
		double elapsedTotal = 0;
		double elapsedLastCall = RCM_MAX_DT_ROBOTS;
		situation->deltaTime = RCM_MAX_DT_SIMU;
		do {
			if (elapsedLastCall >= RCM_MAX_DT_ROBOTS) {
				// Basic control updates
				car->ctrl.gear = DrivingUtil::getGear(*car);
				car->ctrl.brakeCmd = DrivingUtil::getBrake(*car);
				if (car->ctrl.brakeCmd == 0.0) {
					car->ctrl.accelCmd = DrivingUtil::getAccel(*car);
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

		// Evolve real situation
		elapsedTotal = 0;
		elapsedLastCall = RCM_MAX_DT_ROBOTS;
		situation->deltaTime = RCM_MAX_DT_SIMU;
		do {
			if (elapsedLastCall >= RCM_MAX_DT_ROBOTS) {
				// Basic control updates
				realCar->ctrl.gear = DrivingUtil::getGear(*realCar);
				realCar->ctrl.brakeCmd = DrivingUtil::getBrake(*realCar);
				if (realCar->ctrl.brakeCmd == 0.0) {
					realCar->ctrl.accelCmd = DrivingUtil::getAccel(*realCar);
				} else {
					realCar->ctrl.accelCmd = 0.0;
				}
				elapsedLastCall = 0;
			}
			raceEngineInfo._reSimItf.update(&realSituation, RCM_MAX_DT_SIMU, -1);
			realSituation.currentTime += RCM_MAX_DT_SIMU;
			elapsedTotal += RCM_MAX_DT_SIMU;
			elapsedLastCall += RCM_MAX_DT_SIMU;
		} while (elapsedTotal <= STEER_ACTION_FREQ);

		Observation obs = Observation(*situation, 0, 1, actions);
		Observation obs1 = Observation(realSituation, 0, 1, actions);
	
		nextState = State{ nextState };
	}
}

inline
bool TorcsSimulator::simulate(const State& state, unsigned actionIndex, State& nextState, double& reward,unsigned depth) const
{
	Observation observation;
	bool stop = simulate(state, actionIndex, nextState, observation, reward, depth);
	return stop;
}

inline
bool TorcsSimulator::transform(const State& state, const Observation& observation, State& transformedState) const
{
	transformedState = state;
	transformedState.modelState = DriverModel::sampleState(driverActions, RANDOM, state.modelState.action);
	return true;
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