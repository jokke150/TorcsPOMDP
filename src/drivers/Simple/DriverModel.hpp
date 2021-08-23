#include <raceman.h>
#include <robottools.h>

#include "Random.hpp"
#include "Discretizer.hpp"
#include "Constants.h"

using std::vector;

namespace pomdp
{

typedef float Action;

struct TorcsState
{
    TorcsState(tSituation& s);
    TorcsState(float angle, double currentTime, float steerLock);
    float angle;
    double currentTime;
    float steerLock;
};

inline
TorcsState::TorcsState(tSituation& s) {
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
    DriverModel(vector<Action>& driverActions);

    DriverModelState getState();
    void setState(DriverModelState modelState);
    float getAction();
    void update(TorcsState& torcsState);

    static void updateInPlace(const TorcsState& torcsState, DriverModelState& modelState, vector<Action>& driverActions);
    static DriverModelState sampleState(vector<Action>& driverActions);
private:
    DriverModelState state;
    vector<Action>& driverActions;
};

inline 
DriverModel::DriverModel(vector<Action>& driverActions) : driverActions{ driverActions } {
    bool isDistracted = utils::RANDOM.getBool();
    unsigned numActionsRemaining;
    float action;
    if (isDistracted) {
        numActionsRemaining = utils::RANDOM(MAX_DISTRACTED_ACTIONS + 1);
        numActionsRemaining = numActionsRemaining < MIN_DISTRACTED_ACTIONS ? MIN_DISTRACTED_ACTIONS : numActionsRemaining;
        action = driverActions[utils::RANDOM(driverActions.size())];
    } else {
        numActionsRemaining = utils::RANDOM(MAX_ATTENTIVE_ACTIONS + 1);
        numActionsRemaining = numActionsRemaining < MIN_ATTENTIVE_ACTIONS ? MIN_ATTENTIVE_ACTIONS : numActionsRemaining;
        action = -2;
    }

    state = DriverModelState{isDistracted, numActionsRemaining, action };
}

inline 
DriverModelState DriverModel::getState() {
    return state;
}

inline 
void DriverModel::setState(DriverModelState modelState) {
    state = modelState;
}

inline 
float DriverModel::getAction() {
    return state.action;
}

inline 
void DriverModel::update(TorcsState& torcsState) {
    updateInPlace(torcsState, state, driverActions);
}


inline 
void DriverModel::updateInPlace(const TorcsState& torcsState, DriverModelState& modelState, vector<Action>& driverActions) {
    if (modelState.numActionsRemaining == 0) {
        if (modelState.isDistracted) {
            modelState.numActionsRemaining = utils::RANDOM(MAX_DISTRACTED_ACTIONS + 1);
            modelState.numActionsRemaining = modelState.numActionsRemaining < MIN_DISTRACTED_ACTIONS ? 
                MIN_DISTRACTED_ACTIONS : modelState.numActionsRemaining;
            modelState.isDistracted = false;
        } else {
            modelState.numActionsRemaining = utils::RANDOM(MAX_ATTENTIVE_ACTIONS + 1);
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
    }
    modelState.numActionsRemaining--;
}

inline 
DriverModelState DriverModel::sampleState(vector<Action>& driverActions) {
    bool isDistracted = utils::RANDOM.getBool();
    // double duration;
    unsigned numActionsRemaining;
    float action;
    if (isDistracted) {
        // duration = utils::RANDOM(maxDistractionDuration);
        // duration = duration < minDistractionDuration ? minDistractionDuration : duration;
        // duration = utils::RANDOM.getSigned(minAttentionDuration, maxDistractionDuration);
        // duration = utils::Discretizer::discretize(durationBins, duration);
        numActionsRemaining = utils::RANDOM(MAX_DISTRACTED_ACTIONS + 1); // Can be less than MIN_DISTRACTED_ACTIONS here
        action = driverActions[utils::RANDOM(driverActions.size())];
    } else {
        // duration = utils::RANDOM(maxAttentionDuration);
        // duration = duration < minAttentionDuration ? minAttentionDuration : duration;
        // duration = utils::RANDOM.getSigned(minAttentionDuration, maxAttentionDuration);
        // duration = utils::Discretizer::discretize(durationBins, duration);
        numActionsRemaining = utils::RANDOM(MAX_ATTENTIVE_ACTIONS + 1); // Can be less than MIN_ATTENTIVE_ACTIONS here
        action = -2;
    }

    return DriverModelState{isDistracted, numActionsRemaining, action };
}

// inline
// const vector<float> DriverModel::durationBins = [](){ 
//     vector<float> bins;
//     int numBins = (maxAttentionDuration - minAttentionDuration) * 60 / 10; // In 1/10 second bins
//     bins.reserve(numBins);
//     for (int i = 1; i <= numBins; i++) {
//         bins.push_back(1 + (((maxAttentionDuration - minAttentionDuration) / numBins ) * i)); // TODO: Account for distracted
//     }
//     return bins;
// }();

}