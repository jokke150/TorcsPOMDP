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
    TorcsState(float angle, double currentTime, float steerLock);
    float angle;
    double currentTime;
    float steerLock;
};

inline
TorcsState::TorcsState(const tSituation& s) {
    tCarElt* car = s.cars[0];
    const float SC = 1.0;
    angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI
    angle -= SC * car->_trkPos.toMiddle / car->_trkPos.seg->width;
    currentTime = s.currentTime;
    steerLock = car->_steerLock;
}

inline
TorcsState::TorcsState(float angle, double currentTime, float steerLock) : angle{ angle }, currentTime{ currentTime }, steerLock{ steerLock } {}

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
    if (modelState.numActionsRemaining == 0) {
        if (modelState.isDistracted) {
            modelState.numActionsRemaining = rng(MAX_DISTRACTED_ACTIONS + 1);
            modelState.numActionsRemaining = modelState.numActionsRemaining < MIN_DISTRACTED_ACTIONS ? 
                MIN_DISTRACTED_ACTIONS : modelState.numActionsRemaining;
            modelState.isDistracted = false;
        } else {
            modelState.numActionsRemaining = rng(MAX_ATTENTIVE_ACTIONS + 1);
            modelState.numActionsRemaining = modelState.numActionsRemaining < MIN_ATTENTIVE_ACTIONS ? 
                MIN_ATTENTIVE_ACTIONS : modelState.numActionsRemaining;
            modelState.isDistracted = true;
        }
    }

    if (modelState.isDistracted) {
        // Driver is distracted -> repeat last action
    } else {
        // Driver is attentive -> steer to middle
        // TODO: Add more sophisticated driving behavior?
        modelState.action = utils::Discretizer::discretize(driverActions, torcsState.angle / torcsState.steerLock);
        // modelState.action = DrivingUtil::getOptimalSteer(car)
        // getOptimalSteer
    }
    modelState.numActionsRemaining--;
}

inline 
DriverModelState DriverModel::sampleState(vector<Action>& driverActions, RandomNumberGenerator& rng, bool initial = false) 
{
    bool isDistracted = rng.getBool();
    float action;
    unsigned numActionsRemaining;
    if (isDistracted) {
        numActionsRemaining = rng(MAX_DISTRACTED_ACTIONS + 1); 
        if (initial) {
            // Can be less than MIN_DISTRACTED_ACTIONS otherwise
            numActionsRemaining = numActionsRemaining < MIN_DISTRACTED_ACTIONS ? MIN_DISTRACTED_ACTIONS : numActionsRemaining;
        }
        action = driverActions[rng(driverActions.size())];
    } else {
        numActionsRemaining = rng(MAX_ATTENTIVE_ACTIONS + 1);
        if (initial) {
            // Can be less than MIN_ATTENTIVE_ACTIONS otherwise
            numActionsRemaining = numActionsRemaining < MIN_ATTENTIVE_ACTIONS ? MIN_ATTENTIVE_ACTIONS : numActionsRemaining;
        }
        action = -INFINITY;
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

// inline
// const vector<float> DriverModel::durationBins = []()
// { 
//     vector<float> bins;
//     int numBins = (maxAttentionDuration - minAttentionDuration) * 60 / 10; // In 1/10 second bins
//     bins.reserve(numBins);
//     for (int i = 1; i <= numBins; i++) {
//         bins.push_back(1 + (((maxAttentionDuration - minAttentionDuration) / numBins ) * i)); // TODO: Account for distracted
//     }
//     return bins;
// }();

}