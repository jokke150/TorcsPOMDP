#include <vector>
#include <boost/functional/hash.hpp>

#include <raceman.h>
#include <car.h> 

#include "MonteCarloSimulator.hpp"
#include "Random.hpp"
#include "Pomcp.hpp"
#include "TorcsPomdp.hpp"
#include "Discretizer.hpp"



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
	* @param actions: Reference to the action space.
	* @param raceEngineInfo: Information about race engine, e.g. the track. Needed for init of generative model.
	* @param initTorcsState: Reference to the initial torcs state
    * Create a new simulator
    */
	TorcsSimulator(const std::vector<Action>& actions, tSituation initTorcsState, tRmInfo& raceEngineInfo);
	virtual ~TorcsSimulator();
	virtual double getDiscount() const {return DISCOUNT;}
	virtual State& sampleInitialState(State& state) const;
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward,unsigned depth) const;
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward,unsigned depth) const;
	virtual unsigned getNumActions() const {return actions.size();}
	virtual const Action& getAction(unsigned actionIndex) const {return actions[actionIndex];}
	virtual bool allActionsAreValid(const State& state) const {return true;}
	virtual bool isValidAction(const State& state, unsigned actionIndex) const {return true;}

	void test(const State& origState) const; // TODO: Remove
private:
	const std::vector<Action>& actions;
	tRmInfo& raceEngineInfo;
	tSituation initTorcsState;
	
	// Generative model
	tModList *modList = 0;
	tSimItf genModel;
	void loadGenModel();
};

inline
TorcsSimulator::TorcsSimulator(const std::vector<Action>& actions, tSituation initTorcsState, tRmInfo& raceEngineInfo) 
	: actions{actions}, raceEngineInfo{raceEngineInfo}, initTorcsState{initTorcsState} 
{
	loadGenModel();
}

inline
TorcsSimulator::~TorcsSimulator() {}

inline
State& TorcsSimulator::sampleInitialState(State& state) const
{
	DriverModelState modelState = DriverModel::sampleState();
	state = State{ initTorcsState, modelState };
	state.torcsState.currentTime += RCM_MAX_DT_ROBOTS; // Is initially set wrong.
	state.torcsState.deltaTime = RCM_MAX_DT_SIMU; // Is initially set wrong.
	return state;	
}

inline
bool TorcsSimulator::simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward,unsigned depth) const
{
	// test(state);
	
	
	const Action& agentAction = getAction(actionIndex);
	
	reward = RewardCalculator::reward(state, agentAction); // TODO: Is this placed here correctly?
	
	nextState = State{ state };
	tCarElt* car = nextState.torcsState.cars[0];
	
	// Get driver's action
	TorcsState torcsState{ nextState.torcsState };
	DriverModel::updateInPlace(torcsState, nextState.modelState);
	float driverAction = utils::Discretizer::discretize(actions, nextState.modelState.action);

	// Combine steering actions
    car->_steerCmd = utils::Discretizer::discretize(actions, driverAction + agentAction) ;

	genModel.config(car, &raceEngineInfo);
	double elapsed = 0;
	tSituation* situation = &nextState.torcsState;
	situation->deltaTime = RCM_MAX_DT_SIMU;
	while (elapsed < RCM_MAX_DT_ROBOTS) {
		genModel.update(situation, RCM_MAX_DT_SIMU, -1);
		situation->currentTime += RCM_MAX_DT_SIMU;
		elapsed += RCM_MAX_DT_SIMU;
	}
	observation = Observation{ nextState, driverAction};
    return nextState.isTerminal();
}

inline
void TorcsSimulator::test(const State& origState) const {
	State nextState = State{ origState };
	for (int i = 0; i < 1000; i++) {
		tCarElt* car = nextState.torcsState.cars[0];
		const float SC = 1.0;
		float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
		NORM_PI_PI(angle); // normalize the angle between -PI and + PI
		angle -= SC * car->_trkPos.toMiddle / car->_trkPos.seg->width;
		float steerLock = car->_steerLock;
    	car->_steerCmd = angle / steerLock;

		genModel.config(car, &raceEngineInfo);
		double elapsed = 0;
		tSituation* situation = &nextState.torcsState;
		situation->deltaTime = RCM_MAX_DT_SIMU;
		while (elapsed < RCM_MAX_DT_ROBOTS) {
			genModel.update(situation, RCM_MAX_DT_SIMU, -1);
			situation->currentTime += RCM_MAX_DT_SIMU;
			elapsed += RCM_MAX_DT_SIMU;
		}
		nextState = State{ nextState };
	}

	State nextState1 = State{ origState };
	tCarElt* car = nextState1.torcsState.cars[0];
	genModel.config(nextState1.torcsState.cars[0], &raceEngineInfo);
	for (int i = 0; i < 1000; i++) {
		const float SC = 1.0;
		float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
		NORM_PI_PI(angle); // normalize the angle between -PI and + PI
		angle -= SC * car->_trkPos.toMiddle / car->_trkPos.seg->width;
		float steerLock = car->_steerLock;
    	car->_steerCmd = angle / steerLock;

		genModel.config(car, &raceEngineInfo);
		double elapsed = 0;
		tSituation* situation = &nextState1.torcsState;
		situation->deltaTime = RCM_MAX_DT_SIMU;
		while (elapsed < RCM_MAX_DT_ROBOTS) {
			genModel.update(situation, RCM_MAX_DT_SIMU, -1);
			situation->currentTime += RCM_MAX_DT_SIMU;
			elapsed += RCM_MAX_DT_SIMU;
		}		
	}

	int x = 1;
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