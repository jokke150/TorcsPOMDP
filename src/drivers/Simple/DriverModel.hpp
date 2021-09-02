#include <raceman.h>
#include <robottools.h>

#include "Random.hpp"
#include "Discretizer.hpp"
#include "Constants.h"
#include "DrivingUtil.h"

using std::vector;
using utils::RandomNumberGenerator;

namespace pomdp
{

typedef float Action;

struct TorcsState
{
    TorcsState(const tSituation& s);
    TorcsState(tCarElt& car, double currentTime);
    double currentTime;
    tCarElt car;
};

inline
TorcsState::TorcsState(const tSituation& s) {
    car = *s.cars[0];
    currentTime = s.currentTime;
}

inline
TorcsState::TorcsState(tCarElt& car, double currentTime) : car{ car }, currentTime{ currentTime } {}

struct DriverModelState
{
    bool isDistracted;
    unsigned numActionsRemaining;
    float action;
};

// TODO: Refactor to abstract class and inherit with SimpleDriverModel
class DriverModel
{
public:
    DriverModel(vector<Action>& driverActions, unsigned randSeed);

    DriverModelState getState();
    void setState(DriverModelState modelState);
    float getAction();
    void update(TorcsState& torcsState);

    static void updateInPlace(const TorcsState& torcsState, DriverModelState& modelState, vector<Action>& driverActions, RandomNumberGenerator& rng);
    static DriverModelState sampleState(vector<Action>& driverActions, RandomNumberGenerator& rng, bool initial);
    static DriverModelState sampleState(vector<Action>& driverActions, RandomNumberGenerator& rng, float action);
private:
    DriverModelState state;
    vector<Action>& driverActions;
    RandomNumberGenerator rng;
};

inline 
DriverModel::DriverModel(vector<Action>& driverActions, unsigned randSeed) : driverActions{ driverActions }, rng{ RandomNumberGenerator(randSeed) }
{
    state = sampleState(driverActions, rng, true);
}

inline 
DriverModelState DriverModel::getState() {
    return state;
}

inline 
void DriverModel::setState(DriverModelState modelState) 
{
    state = modelState;
}

inline 
float DriverModel::getAction() 
{
    return state.action;
}

inline 
void DriverModel::update(TorcsState& torcsState) 
{
    updateInPlace(torcsState, state, driverActions, rng);
}


inline 
void DriverModel::updateInPlace(const TorcsState& torcsState, DriverModelState& modelState, vector<Action>& driverActions, RandomNumberGenerator& rng) 
{
    bool overCorrect = false;
    if (modelState.numActionsRemaining == 0) {
        if (modelState.isDistracted) {
            modelState.numActionsRemaining = rng(MAX_DISTRACTED_ACTIONS + 1);
            modelState.numActionsRemaining = modelState.numActionsRemaining < MIN_DISTRACTED_ACTIONS ? 
                MIN_DISTRACTED_ACTIONS : modelState.numActionsRemaining;
            modelState.isDistracted = false;
            overCorrect = DRIVER_OVER_CORRECT;
        } else {
            modelState.numActionsRemaining = rng(MAX_ATTENTIVE_ACTIONS + 1);
            modelState.numActionsRemaining = modelState.numActionsRemaining < MIN_ATTENTIVE_ACTIONS ? 
                MIN_ATTENTIVE_ACTIONS : modelState.numActionsRemaining;
            modelState.isDistracted = true;
        }
    }

    float& action = modelState.action;
    if (modelState.isDistracted) {
        // Driver is distracted -> repeat last action
        if (DRIVER_ACTION_NOISE) {
            action += action * rng.getReal(-DRIVER_NOISE_DIST_MAX, DRIVER_NOISE_DIST_MAX);
        }
        if (DRIVER_DISCRETE_ACTIONS) {
            action = utils::Discretizer::discretize(driverActions, action);
        }
    } else {
        // Driver is attentive
        action = DrivingUtil::getOptimalSteer(torcsState.car);

        if (overCorrect && action != 0) {
            action += action * rng.getReal(DRIVER_COR_FACTOR_MIN, DRIVER_COR_FACTOR_MAX);
        }
        if (DRIVER_ACTION_NOISE) {
            action += action * rng.getReal(-DRIVER_NOISE_ATT_MAX, DRIVER_NOISE_ATT_MAX);
        }
        if (DRIVER_DISCRETE_ACTIONS) {
            action = utils::Discretizer::discretize(driverActions, action);
        }
    }
    modelState.numActionsRemaining--;
}

inline 
DriverModelState DriverModel::sampleState(vector<Action>& driverActions, RandomNumberGenerator& rng, bool initial = false) 
{
    bool isDistracted = INITIAL_ATTENTIVE ? false : rng.getBool();
    float action;
    unsigned numActionsRemaining;
    if (isDistracted) {
        numActionsRemaining = rng(MAX_DISTRACTED_ACTIONS + 1); 
        if (initial) {
            // Can be less than MIN_DISTRACTED_ACTIONS otherwise
            numActionsRemaining = numActionsRemaining < MIN_DISTRACTED_ACTIONS ? MIN_DISTRACTED_ACTIONS : numActionsRemaining;
        }
        if (DRIVER_DISCRETE_ACTIONS) {
            action = driverActions[rng(driverActions.size())];
        } else{
            action = rng.getReal(-1, 1);
        }
    } else {
        numActionsRemaining = rng(MAX_ATTENTIVE_ACTIONS + 1);
        if (initial) {
            // Can be less than MIN_ATTENTIVE_ACTIONS otherwise
            numActionsRemaining = numActionsRemaining < MIN_ATTENTIVE_ACTIONS ? MIN_ATTENTIVE_ACTIONS : numActionsRemaining;
        }
        action = 0;
    }

    return DriverModelState{isDistracted, numActionsRemaining, action };
}

inline 
DriverModelState DriverModel::sampleState(vector<Action>& driverActions, RandomNumberGenerator& rng, float action) 
{
    DriverModelState state = sampleState(driverActions, rng);
    state.action = action;
    return state;
}

}