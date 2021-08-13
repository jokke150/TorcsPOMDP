#include <raceman.h>
#include <robottools.h>

#include "Random.hpp"
#include "TorcsPomdp.hpp"

namespace pomdp
{

// TODO: Refactor to abstract class and inherit with SimpleDriverModel
class DriverModel
{
public:
    DriverModel(vector<Action>& driverActions);

    DriverModelState getState();
   void setState(DriverModelState modelState);

    float getAction();

    void update(TorcsState& torcsState);

    static DriverModelState sampleState(vector<Action>& driverActions);
    static void updateInPlace(const TorcsState& torcsState, DriverModelState& modelState, vector<Action>& driverActions);
private:
    // static const unsigned minAttentiveActions = 1 / RCM_MAX_DT_ROBOTS; // Actions for one second of simulated time
    // static const unsigned maxAttentiveActions = 5 / RCM_MAX_DT_ROBOTS; // Actions for five seconds of simulated time
    // static const unsigned minDistractedActions = 1 / RCM_MAX_DT_ROBOTS; // Actions for one second of simulated time
    // static const unsigned maxDistractedActions = 5 / RCM_MAX_DT_ROBOTS; // Actions for five seconds of simulated time
    static const unsigned minAttentiveActions = 1; // Actions for one second of simulated time
    static const unsigned maxAttentiveActions = 5; // Actions for five seconds of simulated time
    static const unsigned minDistractedActions = 1; // Actions for one second of simulated time
    static const unsigned maxDistractedActions = 5; // Actions for five seconds of simulated time
    // static const vector<float> durationBins;

    DriverModelState state;
    vector<Action>& driverActions;
};

inline 
DriverModel::DriverModel(vector<Action>& driverActions) : driverActions{ driverActions } {
    bool isDistracted = utils::RANDOM.getBool();
    unsigned numActions;
    float action;
    if (isDistracted) {
        numActions = utils::RANDOM(maxDistractedActions + 1);
        numActions = numActions < minDistractedActions ? minDistractedActions : numActions;
        action = driverActions[utils::RANDOM(driverActions.size())];
    } else {
        numActions = utils::RANDOM(maxAttentiveActions + 1);
        numActions = numActions < minAttentiveActions ? minAttentiveActions : numActions;
        action = -2;
    }

    state = DriverModelState{isDistracted, numActions, action };
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

inline 
DriverModelState DriverModel::sampleState(vector<Action>& driverActions) {
    bool isDistracted = utils::RANDOM.getBool();
    // double duration;
    unsigned numActions;
    float action;
    if (isDistracted) {
        // duration = utils::RANDOM(maxDistractionDuration);
        // duration = duration < minDistractionDuration ? minDistractionDuration : duration;
        // duration = utils::RANDOM.getSigned(minAttentionDuration, maxDistractionDuration);
        // duration = utils::Discretizer::discretize(durationBins, duration);
        numActions = utils::RANDOM(maxDistractedActions + 1); // Can be less than minDistractedActions here
        action = driverActions[utils::RANDOM(driverActions.size())];
    } else {
        // duration = utils::RANDOM(maxAttentionDuration);
        // duration = duration < minAttentionDuration ? minAttentionDuration : duration;
        // duration = utils::RANDOM.getSigned(minAttentionDuration, maxAttentionDuration);
        // duration = utils::Discretizer::discretize(durationBins, duration);
        numActions = utils::RANDOM(maxAttentiveActions + 1); // Can be less than minAttentiveActions here
        action = -2;
    }

    return DriverModelState{isDistracted, numActions, action };
}

inline 
void DriverModel::updateInPlace(const TorcsState& torcsState, DriverModelState& modelState, vector<Action>& driverActions) {
    if (modelState.numActions == 0) {
        if (modelState.isDistracted) {
            modelState.numActions = utils::RANDOM(maxDistractedActions + 1);
            modelState.numActions = modelState.numActions < minDistractedActions ? minDistractedActions : modelState.numActions;
            modelState.isDistracted = false;
        } else {
            modelState.numActions = utils::RANDOM(maxAttentiveActions + 1);
            modelState.numActions = modelState.numActions < minAttentiveActions ? minAttentiveActions : modelState.numActions;
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
    modelState.numActions--;
}

}