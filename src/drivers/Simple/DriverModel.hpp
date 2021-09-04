#include <raceman.h>
#include <robottools.h>

#include "Random.hpp"
#include "Discretizer.hpp"
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
    DriverModel(vector<Action>& driverActions, unsigned randSeed, bool overCorrect, bool noise, bool driverInitAtt);

    DriverModelState getState();
    void setState(DriverModelState modelState);
    float getAction();
    void update(TorcsState& torcsState);

    static void updateInPlace(const TorcsState& torcsState, DriverModelState& modelState, vector<Action>& driverActions, 
                              RandomNumberGenerator& rng, bool overCorrect, bool noise);
    static DriverModelState sampleState(vector<Action>& driverActions, RandomNumberGenerator& rng, bool driverInitAtt, bool initial);
    static DriverModelState sampleState(vector<Action>& driverActions, RandomNumberGenerator& rng, float action);
private:
    DriverModelState state;
    vector<Action>& driverActions;
    RandomNumberGenerator rng;
    bool overCorrect;
     bool noise;
     bool driverInitAtt;
};

inline 
DriverModel::DriverModel(vector<Action>& driverActions, unsigned randSeed, bool overCorrect, bool noise, bool driverInitAtt) 
                        : driverActions{ driverActions }, rng{ RandomNumberGenerator(randSeed) }, 
                          overCorrect{ overCorrect }, noise{ noise }, driverInitAtt{ driverInitAtt }
{
    state = sampleState(driverActions, rng, driverInitAtt, true);
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
    updateInPlace(torcsState, state, driverActions, rng, overCorrect, noise);
}


inline 
void DriverModel::updateInPlace(const TorcsState& torcsState, DriverModelState& modelState, vector<Action>& driverActions, 
                                RandomNumberGenerator& rng, bool overCorrect, bool noise) 
{
    bool overCorrectNow = false;
    if (modelState.numActionsRemaining == 0) {
        if (modelState.isDistracted) {
            modelState.numActionsRemaining = rng(MAX_DISTRACTED_ACTIONS + 1);
            modelState.numActionsRemaining = modelState.numActionsRemaining < MIN_DISTRACTED_ACTIONS ? 
                MIN_DISTRACTED_ACTIONS : modelState.numActionsRemaining;
            modelState.isDistracted = false;
            overCorrectNow = overCorrect;
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
        if (noise) {
            action += action * rng.getReal(-DRIVER_NOISE_DIST_MAX, DRIVER_NOISE_DIST_MAX);
        }
        if (DRIVER_DISCRETE_ACTIONS) {
            action = utils::Discretizer::discretize(driverActions, action);
        }
    } else {
        // Driver is attentive
        action = DrivingUtil::getOptimalSteer(torcsState.car);

        if (overCorrectNow && action != 0) {
            action += action * rng.getReal(DRIVER_COR_FACTOR_MIN, DRIVER_COR_FACTOR_MAX);
        }
        if (noise) {
            action += action * rng.getReal(-DRIVER_NOISE_ATT_MAX, DRIVER_NOISE_ATT_MAX);
        }
        if (DRIVER_DISCRETE_ACTIONS) {
            action = utils::Discretizer::discretize(driverActions, action);
        }
    }
    modelState.numActionsRemaining--;
}

inline 
DriverModelState DriverModel::sampleState(vector<Action>& driverActions, RandomNumberGenerator& rng, bool driverInitAtt, bool initial) 
{
    bool isDistracted = initial && driverInitAtt ? false : rng.getBool();
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
    DriverModelState state = sampleState(driverActions, rng, false, false);
    state.action = action;
    return state;
}

}