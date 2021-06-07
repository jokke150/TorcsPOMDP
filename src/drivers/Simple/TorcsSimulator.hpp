#include <vector>

#include <boost/functional/hash.hpp>
#include "MonteCarloSimulator.hpp"

#include "Random.hpp"
#include "Pomcp.hpp"
#include "TorcsPomdp.hpp"

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
	tSituation lastTorcsState;
	
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
	reward = RewardCalculator::reward(state, action);
	nextState = State{ state };
	tCarElt* car = nextState.torcsState.cars[0];
	genModel.config(car, &raceEngineInfo);
	genModel.update(&nextState.torcsState, RCM_MAX_DT_SIMU, -1);
	observation = Observation{ nextState };
    double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
	return absDistToMiddle >= TERMINAL_OFF_LANE_DIST;
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