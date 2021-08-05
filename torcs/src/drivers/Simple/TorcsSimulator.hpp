#include <vector>
#include <boost/functional/hash.hpp>

#include <raceman.h>
#include <car.h> 

#include "MonteCarloSimulator.hpp"
#include "Random.hpp"
#include "Pomcp.hpp"
#include "DriverModel.hpp"

using namespace pomcp;

namespace pomdp
{

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
	TorcsSimulator(tSituation& situation, tRmInfo& raceEngineInfo);
	virtual ~TorcsSimulator();
	virtual double getDiscount() const {return DISCOUNT;}
	virtual State& sampleInitialState(State& state) const;
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward,unsigned depth) const;
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward,unsigned depth) const;
	virtual unsigned getNumActions() const {return Observation::actions.size();}
	virtual const Action& getAction(unsigned actionIndex) const {return Observation::actions[actionIndex];}
	virtual bool allActionsAreValid(const State& state) const {return true;}
	virtual bool isValidAction(const State& state, unsigned actionIndex) const {return true;}

	void test(const State& origState) const; // TODO: Remove
private:
	tRmInfo& raceEngineInfo;
	tSituation& realSituation;
	
	// Generative model
	tModList *modList = 0;
	tSimItf genModel;
	void loadGenModel();
};

inline
TorcsSimulator::TorcsSimulator(tSituation& situation, tRmInfo& raceEngineInfo) 
	: raceEngineInfo{raceEngineInfo}, realSituation{situation}
{
	loadGenModel();
}

inline
TorcsSimulator::~TorcsSimulator() {}

inline   
State& TorcsSimulator::sampleInitialState(State& state) const
{
	tCar initEnvState;
	raceEngineInfo._reSimItf.getState(&initEnvState);
	DriverModelState modelState = DriverModel::sampleState(realSituation.currentTime);
	state = State{ realSituation, initEnvState, modelState, 0 };
	return state;
}

inline
bool TorcsSimulator::simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward, unsigned depth) const
{
	// test(state);

	const Action& agentAction = getAction(actionIndex);
	
	nextState = state;
	tSituation* situation = &nextState.situation;

	// Get driver's action
	TorcsState torcsState{ *situation };
	DriverModel::updateInPlace(torcsState, nextState.modelState);
	// float driverAction = utils::Discretizer::discretize(actions, nextState.modelState.action);
	float driverAction = nextState.modelState.action;

	// Combine steering actions
    // situation->cars[0]->_steerCmd = utils::Discretizer::discretize(actions, driverAction + agentAction);
	// situation->cars[0]->_steerCmd = agentAction;
	situation->cars[0]->_steerCmd = std::max(std::min(driverAction + agentAction, 1.0f), -1.0f);

	// Set simulator's internal car state
	genModel.setState(&nextState.car);
	
	// Simulate situation
	double elapsed = 0;
	situation->deltaTime = RCM_MAX_DT_SIMU;
	while (elapsed <= RCM_MAX_DT_ROBOTS + RCM_MAX_DT_SIMU) {
		genModel.update(situation, RCM_MAX_DT_SIMU, -1);
		situation->currentTime += RCM_MAX_DT_SIMU;
		elapsed += RCM_MAX_DT_SIMU;
	}

	// Set reward after obtaining next state as it is based upon it
	reward = RewardCalculator::reward(*situation, agentAction);

	nextState.actionsCount++;
	observation = Observation{ *situation, driverAction, nextState.actionsCount};
    return nextState.isTerminal();
}

inline
void TorcsSimulator::test(const State& origState) const {
	State nextState = origState;
	tSituation* situation = &nextState.situation;
	tCarElt* carElt = nextState.situation.cars[0];
	for (int i = 0; i < 10000; i++) {
		const float SC = 1.0;
		float angle =  RtTrackSideTgAngleL(&(carElt->_trkPos)) - carElt->_yaw;
		NORM_PI_PI(angle); // normalize the angle between -PI and + PI
		angle -= SC * carElt->_trkPos.toMiddle / carElt->_trkPos.seg->width;
		float steerLock = carElt->_steerLock;
    	carElt->_steerCmd = angle / steerLock;

		realSituation.cars[0]->_steerCmd = carElt->_steerCmd;

		// Set simulator's internal car state
		genModel.setState(&nextState.car);

		double elapsed = 0;
		while (elapsed <= RCM_MAX_DT_ROBOTS + RCM_MAX_DT_SIMU) {
			genModel.update(situation, RCM_MAX_DT_SIMU, -1);
			situation->currentTime += RCM_MAX_DT_SIMU;
			elapsed += RCM_MAX_DT_SIMU;
		}

		elapsed = 0;
		while (elapsed <= RCM_MAX_DT_ROBOTS + RCM_MAX_DT_SIMU) {
			raceEngineInfo._reSimItf.update(&realSituation, RCM_MAX_DT_SIMU, -1);
			realSituation.currentTime += RCM_MAX_DT_SIMU;
			elapsed += RCM_MAX_DT_SIMU;
		}

		tCar realCar;
		raceEngineInfo._reSimItf.getState(&realCar); 

		Observation obs = Observation(*situation, 0, 1);
		Observation obs1 = Observation(realSituation, 0, 1);
	
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