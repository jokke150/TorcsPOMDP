#include "Random.hpp"
#include <raceman.h>

namespace pomdp
{

struct DriverModelState
{
    bool isDistracted;
    int timeEpisodeEnd;
    float action;
    float lastAction;
};

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
    angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    currentTime = s.currentTime;
    steerLock = car->_steerLock;
}

inline
TorcsState::TorcsState(float angle, double currentTime, float steerLock) : angle{ angle }, currentTime{ currentTime }, steerLock{ steerLock } {}

// TODO: Refactor to abstract class and inherit with SimpleDriverModel
class DriverModel
{
public:
    DriverModel();

    DriverModelState getState();
   void setState(DriverModelState modelState);

    float getAction();
    float getLastAction();

    void update(TorcsState& torcsState);

    static DriverModelState sampleState();
    static void updateInPlace(TorcsState& torcsState, DriverModelState& modelState);
private:
    static const int minAttentionDuration = 1; // Minimum duration of attention episode in ms
    static const int maxAttentionDuration = 1; // Maximum duration of attention episode in ms
    static const int minDistractionDuration = 1; // Minimum duration of distration episode in ms
    static const int maxDistractionDuration = 1; // Maximum duration of distration episode in ms

    DriverModelState state;
};

inline 
DriverModel::DriverModel() {
    state = DriverModelState{false, -1, -2, -2};
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
float DriverModel::getLastAction() {
    return state.lastAction;
}

inline 
void DriverModel::update(TorcsState& torcsState) {
    updateInPlace(torcsState, state);
}

inline 
DriverModelState DriverModel::sampleState() {
    return DriverModelState{utils::RANDOM.getBool(), -1, (float) utils::RANDOM.getSigned(-1.0, 1.0), (float) utils::RANDOM.getSigned(-1.0, 1.0)};
}

inline 
void DriverModel::updateInPlace(TorcsState& torcsState, DriverModelState& modelState) {
    if (torcsState.currentTime >= modelState.timeEpisodeEnd) {
        if (modelState.isDistracted) {
            int duration = utils::RANDOM(maxAttentionDuration);
            modelState.timeEpisodeEnd = duration > minAttentionDuration ? duration : minAttentionDuration;
            modelState.isDistracted = false;
        } else {
            int duration = utils::RANDOM(maxDistractionDuration);
            modelState.timeEpisodeEnd = duration > minDistractionDuration ? duration : minDistractionDuration;
            modelState.isDistracted = true;
        }
    }

    if (modelState.isDistracted) {
        // Driver is distracted -> repeat last action
        modelState.lastAction = modelState.action;
    } else {
        // Driver is attentive -> steer to middle
        // TODO: Add more sophisticated driving behavior?
        modelState.lastAction = modelState.action;
        modelState.action = torcsState.angle / torcsState.steerLock;
    }
}

}