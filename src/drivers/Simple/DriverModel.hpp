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
    // return state.action;
    return std::max(std::min(state.action, 0.5f), -0.5f);
}

inline 
void DriverModel::update(TorcsState& torcsState) {
    updateInPlace(torcsState, state);
}

inline 
DriverModelState DriverModel::sampleState(double currentTime) {
    bool isDistracted = utils::RANDOM.getBool();
    double duration;
    float action;
    if (isDistracted) {
        duration = utils::RANDOM(maxDistractionDuration);
        duration = duration < minDistractionDuration ? minDistractionDuration : duration;
        // duration = utils::RANDOM.getSigned(minDistractionDuration, maxDistractionDuration);
        action = utils::RANDOM(Observation::actions.size());
    } else {
        duration = utils::RANDOM(maxAttentionDuration);
        duration = duration < minAttentionDuration ? minAttentionDuration : duration;
        // duration = utils::RANDOM.getSigned(minAttentionDuration, maxAttentionDuration);
        action = -2;
    }
    
    return DriverModelState{isDistracted, currentTime + duration, action };
}

inline 
void DriverModel::updateInPlace(const TorcsState& torcsState, DriverModelState& modelState) {
    if (torcsState.currentTime >= modelState.timeEpisodeEnd) {
        if (modelState.isDistracted) {
            double duration = utils::RANDOM(maxAttentionDuration);
            duration = duration < minAttentionDuration ? minAttentionDuration : duration;
            // double duration = utils::RANDOM.getSigned(minAttentionDuration, maxAttentionDuration);
            modelState.timeEpisodeEnd = torcsState.currentTime + duration;
            modelState.isDistracted = false;
        } else {
            double duration = utils::RANDOM(maxDistractionDuration);
            duration = duration < minDistractionDuration ? minDistractionDuration : duration;
            // double duration = utils::RANDOM.getSigned(minDistractionDuration, maxDistractionDuration);
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