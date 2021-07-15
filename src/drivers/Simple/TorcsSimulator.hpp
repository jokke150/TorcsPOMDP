#include <vector>
#include <boost/functional/hash.hpp>

#include <raceman.h>
#include <car.h> 

#include "MonteCarloSimulator.hpp"
#include "Random.hpp"
#include "Pomcp.hpp"
#include "TorcsPomdp.hpp"

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
	* @param initSituation: Reference to the initial torcs state
    * Create a new simulator
    */
	TorcsSimulator(const std::vector<Action>& actions, tSituation& initSituation, tRmInfo& raceEngineInfo);
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
	tCar initEnvState;
	tSituation initSituation;
	tSituation& initRef;
	
	// Generative model
	tModList *modList = 0;
	tSimItf genModel;
	void loadGenModel();
};

inline
TorcsSimulator::TorcsSimulator(const std::vector<Action>& actions, tSituation& initSituation, tRmInfo& raceEngineInfo) 
	: actions{actions}, raceEngineInfo{raceEngineInfo}, initSituation{initSituation}, initRef{ initSituation }
{
	raceEngineInfo._reSimItf.getState(&initEnvState);
	loadGenModel();
}

inline
TorcsSimulator::~TorcsSimulator() {}

inline
State& TorcsSimulator::sampleInitialState(State& state) const
{
	DriverModelState modelState = DriverModel::sampleState();
	tSituation situation{ initSituation };

	tCar car{ initEnvState };
	tCarElt* carElt = situation.cars[0];

	// Update the pointers in the simulator's internal car state to match the new situation
	car.carElt = carElt;
	car.ctrl   = &carElt->ctrl;
    car.params = carElt->_carHandle;

	// Update transmission pointers so that they point to this car's values
	tTransmission *trans = &(car.transmission);

	/* Link between the differentials */
	for (int j = 0; j < 2; j++) {
		trans->differential[TRANS_FRONT_DIFF].inAxis[j]  = &(car.wheel[j].feedBack);
		trans->differential[TRANS_FRONT_DIFF].outAxis[j] = &(car.wheel[j].in);
	}

	for (int j = 0; j < 2; j++) {
		trans->differential[TRANS_REAR_DIFF].inAxis[j]  = &(car.wheel[2+j].feedBack);
		trans->differential[TRANS_REAR_DIFF].outAxis[j] = &(car.wheel[2+j].in);
	}

	trans->differential[TRANS_CENTRAL_DIFF].inAxis[0]  = &(trans->differential[TRANS_FRONT_DIFF].feedBack);
	trans->differential[TRANS_CENTRAL_DIFF].outAxis[0] = &(trans->differential[TRANS_FRONT_DIFF].in);
	
	trans->differential[TRANS_CENTRAL_DIFF].inAxis[1]  = &(trans->differential[TRANS_REAR_DIFF].feedBack);
	trans->differential[TRANS_CENTRAL_DIFF].outAxis[1] = &(trans->differential[TRANS_REAR_DIFF].in);

	state = State{ situation, car, modelState, 0 };
	return state;
}

inline
bool TorcsSimulator::simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward, unsigned depth) const
{
	// test(state);
	
	const Action& agentAction = getAction(actionIndex);
	
	nextState = State{ state };
	tCar* car = &nextState.car;
	tCarElt* carElt = nextState.situation.cars[0];
	tSituation* situation = &nextState.situation;

	// Update the pointers in the simulator's internal car state to match the new situation
    car->carElt = carElt;
    car->ctrl   = &carElt->ctrl;
    car->params = carElt->_carHandle;

	// Update transmission pointers so that they point to this car's values
	tTransmission *trans = &(car->transmission);

	/* Link between the differentials */
	for (int j = 0; j < 2; j++) {
		trans->differential[TRANS_FRONT_DIFF].inAxis[j]  = &(car->wheel[j].feedBack);
		trans->differential[TRANS_FRONT_DIFF].outAxis[j] = &(car->wheel[j].in);
	}

	for (int j = 0; j < 2; j++) {
		trans->differential[TRANS_REAR_DIFF].inAxis[j]  = &(car->wheel[2+j].feedBack);
		trans->differential[TRANS_REAR_DIFF].outAxis[j] = &(car->wheel[2+j].in);
	}

	trans->differential[TRANS_CENTRAL_DIFF].inAxis[0]  = &(trans->differential[TRANS_FRONT_DIFF].feedBack);
	trans->differential[TRANS_CENTRAL_DIFF].outAxis[0] = &(trans->differential[TRANS_FRONT_DIFF].in);
	
	trans->differential[TRANS_CENTRAL_DIFF].inAxis[1]  = &(trans->differential[TRANS_REAR_DIFF].feedBack);
	trans->differential[TRANS_CENTRAL_DIFF].outAxis[1] = &(trans->differential[TRANS_REAR_DIFF].in);
	
	// Get driver's action
	TorcsState torcsState{ *situation };
	DriverModel::updateInPlace(torcsState, nextState.modelState);
	float driverAction = utils::Discretizer::discretize(actions, nextState.modelState.action);

	// Combine steering actions
    // car->_steerCmd = utils::Discretizer::discretize(actions, driverAction + agentAction) ;
	carElt->_steerCmd = agentAction;

	// Set simulator's internal car state
	genModel.setState(car);
	
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

	// Update next state's car state
	// genModel.getState(car); 

	nextState.actionsCount++;
	observation = Observation{ *situation, driverAction, nextState.actionsCount};
    return nextState.isTerminal();
}

inline
void TorcsSimulator::test(const State& origState) const {
	State nextState = State{ origState };
	for (int i = 0; i < 10000; i++) {
		tCar* car = &nextState.car;
		tCarElt* carElt = nextState.situation.cars[0];
		tSituation* situation = &nextState.situation;

		// Update the pointers in the simulator's internal car state to match the new situation
		car->carElt = carElt;
		car->ctrl   = &carElt->ctrl;
		car->params = carElt->_carHandle;

		// Update transmission pointers so that they point to this car's values
		tTransmission *trans = &(car->transmission);

		/* Link between the differentials */
		for (int j = 0; j < 2; j++) {
			trans->differential[TRANS_FRONT_DIFF].inAxis[j]  = &(car->wheel[j].feedBack);
			trans->differential[TRANS_FRONT_DIFF].outAxis[j] = &(car->wheel[j].in);
		}

		for (int j = 0; j < 2; j++) {
			trans->differential[TRANS_REAR_DIFF].inAxis[j]  = &(car->wheel[2+j].feedBack);
			trans->differential[TRANS_REAR_DIFF].outAxis[j] = &(car->wheel[2+j].in);
		}

		trans->differential[TRANS_CENTRAL_DIFF].inAxis[0]  = &(trans->differential[TRANS_FRONT_DIFF].feedBack);
		trans->differential[TRANS_CENTRAL_DIFF].outAxis[0] = &(trans->differential[TRANS_FRONT_DIFF].in);
		
		trans->differential[TRANS_CENTRAL_DIFF].inAxis[1]  = &(trans->differential[TRANS_REAR_DIFF].feedBack);
		trans->differential[TRANS_CENTRAL_DIFF].outAxis[1] = &(trans->differential[TRANS_REAR_DIFF].in);

		const float SC = 1.0;
		float angle =  RtTrackSideTgAngleL(&(carElt->_trkPos)) - carElt->_yaw;
		NORM_PI_PI(angle); // normalize the angle between -PI and + PI
		angle -= SC * carElt->_trkPos.toMiddle / carElt->_trkPos.seg->width;
		float steerLock = carElt->_steerLock;
    	carElt->_steerCmd = angle / steerLock;

		initRef.cars[0]->_steerCmd = carElt->_steerCmd;

		// Set simulator's internal car state
		genModel.setState(car);

		double elapsed = 0;
		while (elapsed <= RCM_MAX_DT_ROBOTS + RCM_MAX_DT_SIMU) {
			genModel.update(situation, RCM_MAX_DT_SIMU, -1);
			situation->currentTime += RCM_MAX_DT_SIMU;
			elapsed += RCM_MAX_DT_SIMU;
		}

		elapsed = 0;
		while (elapsed <= RCM_MAX_DT_ROBOTS + RCM_MAX_DT_SIMU) {
			raceEngineInfo._reSimItf.update(&initRef, RCM_MAX_DT_SIMU, -1);
			initRef.currentTime += RCM_MAX_DT_SIMU;
			elapsed += RCM_MAX_DT_SIMU;
		}

		tCar realCar;
		raceEngineInfo._reSimItf.getState(&realCar); 

		Observation obs = Observation(*situation, 0, 1);
		Observation obs1 = Observation(initRef, 0, 1);
	
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
		// genModel.config(initSituation.cars[0], &raceEngineInfo);
		// genModel.setState(&initEnvState);
    }
}

}