#include "Random.hpp"

namespace pomdp
{

struct DriverModelState
{
    bool isDistracted;
    int timeEpisodeEnd;
    float lastAction;
};

struct TorcsState
{
    float angle;
    double currentTime;
    float& steerLock;
};

// TODO: Refactor to abstract class and inherit with SimpleDriverModel
class DriverModel
{
public:
    DriverModelState getState();
    DriverModelState setState(DriverModelState modelState);
    float updateAndGetAction(TorcsState torcsState);

    static void updateInPlace(TorcsState& torcsState, DriverModelState& modelState);
private:
    static const int minAttentionDuration = 1; // Minimum duration of attention episode in ms
    static const int maxAttentionDuration = 1; // Maximum duration of attention episode in ms
    static const int minDistractionDuration = 1; // Minimum duration of distration episode in ms
    static const int maxDistractionDuration = 1; // Maximum duration of distration episode in ms

    DriverModelState state;
};

inline DriverModelState DriverModel::getState() {
    return state;
}

inline DriverModelState DriverModel::setState(DriverModelState modelState) {
    state = modelState;
}

inline float DriverModel::updateAndGetAction(TorcsState torcsState) {
    if (torcsState.currentTime >= state.timeEpisodeEnd) {
        if (state.isDistracted) {
            int duration = utils::RANDOM(maxAttentionDuration);
            state.timeEpisodeEnd = duration > minAttentionDuration ? duration : minAttentionDuration;
            state.isDistracted = false;
        } else {
            int duration = utils::RANDOM(maxDistractionDuration);
            state.timeEpisodeEnd = duration > minDistractionDuration ? duration : minDistractionDuration;
            state.isDistracted = true;
        }
    }

    if (state.isDistracted) {
        return state.lastAction;
    } else {
        // TODO: Add more sophisticated driving behavior?
        float nextAction = torcsState.angle / torcsState.steerLock;
        state.lastAction = nextAction;
        return nextAction;
    }
}

}