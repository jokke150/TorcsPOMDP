#include <raceman.h>
#include <robottools.h>

#include "Random.hpp"

namespace pomdp
{

struct DriverModelState
{
    bool isDistracted;
    double timeEpisodeEnd;
    float action;
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
    const float SC = 1.0;
    angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI
    angle -= SC * car->_trkPos.toMiddle / car->_trkPos.seg->width;
    currentTime = s.currentTime;
    steerLock = car->_steerLock;
}

inline
TorcsState::TorcsState(float angle, double currentTime, float steerLock) : angle{ angle }, currentTime{ currentTime }, steerLock{ steerLock } {}

// TODO: Refactor to abstract class and inherit with SimpleDriverModel
class DriverModel
{
public:
    DriverModel(double currentTime);

    DriverModelState getState();
   void setState(DriverModelState modelState);

    float getAction();

    void update(TorcsState& torcsState);

    static DriverModelState sampleState(double currentTime);
    static void updateInPlace(const TorcsState& torcsState, DriverModelState& modelState);
private:
    static constexpr double minAttentionDuration = 1; // Minimum duration of attention episode in s
    static constexpr double maxAttentionDuration = 5; // Maximum duration of attention episode in s
    static constexpr double minDistractionDuration = 1; // Minimum duration of distration episode in s
    static constexpr double maxDistractionDuration = 5;; // Maximum duration of distration episode in s

    DriverModelState state;
};

inline 
DriverModel::DriverModel(double currentTime) {
    state = sampleState(currentTime);
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
    updateInPlace(torcsState, state);
}

inline 
DriverModelState DriverModel::sampleState(double currentTime) {
    double duration = utils::RANDOM.getSigned(minAttentionDuration, maxAttentionDuration);
    return DriverModelState{ false, currentTime + duration, -2 };
}

inline 
void DriverModel::updateInPlace(const TorcsState& torcsState, DriverModelState& modelState) {
    if (torcsState.currentTime >= modelState.timeEpisodeEnd) {
        if (modelState.isDistracted) {
            double duration = utils::RANDOM.getSigned(minAttentionDuration, maxAttentionDuration);
            modelState.timeEpisodeEnd = torcsState.currentTime + duration;
            modelState.isDistracted = false;
        } else {
            double duration = utils::RANDOM.getSigned(minDistractionDuration, maxDistractionDuration);
            modelState.timeEpisodeEnd = torcsState.currentTime + duration;
            modelState.isDistracted = true;
        }
    }

    if (modelState.isDistracted) {
        // Driver is distracted -> repeat last action
    } else {
        // Driver is attentive -> steer to middle
        // TODO: Add more sophisticated driving behavior?
        modelState.action = torcsState.angle / torcsState.steerLock;
    }
}

}