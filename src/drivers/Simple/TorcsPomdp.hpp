#include<cmath>
#include <boost/functional/hash.hpp>

#include <tgf.h>

#include <raceman.h>

#include "sensors.h"

#include "DriverModel.hpp"

namespace pomdp
{

// HYPERPARAMETERS
// TODO: Tune
#define PLANNING_TIME 10
#define RESAMPLING_TIME 10
#define THRESHOLD 0.01
#define EXPLORATION_CTE 100
#define PARTICLES 1
#define DISCOUNT 0.9999

#define REWARD_CENTER 1
#define PENALTY_OFF_LANE 10.0
#define PENALTY_INTENSITY_EXP 2

#define TERMINAL_OFF_LANE_DIST 1.2

typedef struct State 
{   
    State();
    State(const State& other);
    State(tSituation torcsState, DriverModelState modelState);
    bool isTerminal();

    tSituation torcsState;
    DriverModelState modelState;
} State;

class Observation
{   
public:
    Observation() = default;
    Observation(State& s);
    bool operator==(Observation const& other) const;

    // std::vector<float> trackSensorData;
    float angle;
    // float distToMiddle;

private:
    // static const int numSensors;
    //static const float sensorRange; 
    // static const std::vector<float> trackSensAngle;
    // static Sensors *trackSensors;
    // static const std::vector<float> sensorBins;
    static const int numAngleBins;
    static const std::vector<float> angleBins;
    // static const std::vector<float> middleBins;
};

}

namespace std {
  template <> struct hash<pomdp::Observation>
  {
    size_t operator()(const pomdp::Observation & o) const
    {
        std::size_t seed = 0;
        // boost::hash_combine(seed, o.trackSensorData);
        boost::hash_combine(seed, o.angle);
        // boost::hash_combine(seed, o.distToMiddle);
        return seed;
    }
  };
}

namespace pomdp {

typedef float Action;

class RewardCalculator 
{
public:
    static double reward(const State& s, const Action& a);
private:
    static double rewardPosition(const State& s);
    static double penaltyActionIntensity(const Action& a);
};

}