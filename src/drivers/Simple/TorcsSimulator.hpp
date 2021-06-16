#include <vector>

#include <boost/functional/hash.hpp>
#include "MonteCarloSimulator.hpp"

#include "Random.hpp"
#include "Pomcp.hpp"
#include "TorcsPomdp.hpp"
#include "DriverModel.hpp"

#include <raceman.h>
#include <car.h> 

using namespace pomcp;

namespace pomdp
{

/**
 * class InitBelief
 *
 * A class representing the initial belief.
 */
class InitBelief
{
	public:
		InitBelief(const tSituation& initTorcsState);
		~InitBelief() {}
		State sample() const;
	private:
		std::vector<State> possibleStates;
};

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
	* @param actions: Reference to the action space.
	* @param raceEngineInfo: Information about race engine, e.g. the track. Needed for init of generative model.
	* @param initTorcsState: Reference to the initial torcs state
    * Create a new simulator
    */
	TorcsSimulator(const std::vector<Action>& actions, const tSituation& initTorcsState, tRmInfo& raceEngineInfo);
	virtual ~TorcsSimulator();
	virtual double getDiscount() const {return DISCOUNT;}
	virtual State& sampleInitialState(State& state) const;
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward,unsigned depth) const;
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward,unsigned depth) const;
	virtual unsigned getNumActions() const {return actions.size();}
	virtual const Action& getAction(unsigned actionIndex) const {return actions[actionIndex];}
	virtual bool allActionsAreValid(const State& state) const {return true;}
	virtual bool isValidAction(const State& state, unsigned actionIndex) const {return true;}
	virtual void setInitialBelief(const tSituation& initTorcsState);
private:
	const std::vector<Action>& actions;
	tRmInfo& raceEngineInfo;
	InitBelief initialBelief;
	
	// Generative model
	tModList *modList = 0;
	tSimItf genModel;
	void loadGenModel();
};

inline
InitBelief::InitBelief(const tSituation& initTorcsState)
{
	// TODO: Account for possible inner states of driver model
	State initState = { initTorcsState };
	initState.torcsState.currentTime += RCM_MAX_DT_ROBOTS; // Is initially set wrong.
	initState.torcsState.deltaTime = RCM_MAX_DT_ROBOTS; // Is initially set wrong.
	possibleStates.push_back(initState);
}

/**
* Selects a state from a uniform distribution over possible initial states
*/
inline
State InitBelief::sample() const
{
	State sample = possibleStates[utils::RANDOM(possibleStates.size())];
	return sample;
}

inline
TorcsSimulator::TorcsSimulator(const std::vector<Action>& actions, const tSituation& initTorcsState, tRmInfo& raceEngineInfo) 
	: actions{actions}, raceEngineInfo{raceEngineInfo}, initialBelief{InitBelief(initTorcsState)} 
{
	loadGenModel();
}

inline
TorcsSimulator::~TorcsSimulator() {}

inline
State& TorcsSimulator::sampleInitialState(State& state) const
{
	state = initialBelief.sample();
	return state;	
}

inline
bool TorcsSimulator::simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward,unsigned depth) const
{
	const Action& action = getAction(actionIndex);
	reward = RewardCalculator::reward(state, action); // TODO: Is this placed here correctly?
	nextState = State{ state };
	tCarElt* car = nextState.torcsState.cars[0];
	
	// Get driver's action
	float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI
	TorcsState torcsState{angle, nextState.torcsState.currentTime, car->_steerLock};
	DriverModel::updateInPlace(torcsState, nextState.modelState);

	car->_steerCmd = actions[actionIndex] + nextState.modelState.lastAction;

	// For different RCM_MAX_DT_ROBOTS and RCM_MAX_DT_SIMU update intervals, this avoids floating point error accumulation. 
	// However, this implementation is very inefficient. 
	// Different RCM_MAX_DT_ROBOTS and RCM_MAX_DT_SIMU update intervals should therefore be avoided.
	genModel.config(car, &raceEngineInfo);
	double elapsed = 0;
	tSituation* situation = &nextState.torcsState;
	while (elapsed < RCM_MAX_DT_ROBOTS) {
		genModel.update(situation, RCM_MAX_DT_SIMU, -1);
		situation->currentTime += elapsed;
		situation->deltaTime = elapsed;
		elapsed += RCM_MAX_DT_SIMU;
	}
	observation = Observation{ nextState };
	
    return nextState.isTerminal();
}

inline
bool TorcsSimulator::simulate(const State& state, unsigned actionIndex, State& nextState, double& reward,unsigned depth) const
{
	Observation observation;
	bool stop = simulate(state, actionIndex, nextState, observation, reward, depth);
	return stop;
}

inline 
void TorcsSimulator::setInitialBelief(const tSituation& initTorcsState) 
{
	initialBelief = { initTorcsState };
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