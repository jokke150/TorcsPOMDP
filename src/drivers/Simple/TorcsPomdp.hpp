#include<cmath>
#include <boost/functional/hash.hpp>
#include <tgf.h>
#include <raceman.h>
#include <carstruct.h>

#include "sensors.h"
#include "DriverModel.hpp"
#include "Discretizer.hpp"

namespace pomdp
{

// HYPERPARAMETERS
// TODO: Tune
#define PLANNING_TIME 1
#define RESAMPLING_TIME 1
#define THRESHOLD 0.1
#define EXPLORATION_CTE 100
#define PARTICLES 10000
#define DISCOUNT 0.95

#define REWARD_CENTER 1
#define PENALTY_OFF_LANE 10.0
#define PENALTY_INTENSITY_EXP 2

#define TERMINAL_OFF_LANE_DIST 0.1

typedef struct State 
{   
    State();
    State(const State& other);
    State(tSituation& situation, tCar& car, DriverModelState& modelState, int actionsCount);
    
    bool isTerminal();
    static bool isTerminal(tSituation& situation);

    tSituation situation;
    tCar car;
    DriverModelState modelState;
    int actionsCount;
} State;

class Observation
{   
public:
    Observation() = default;
    Observation(tSituation& situation, float lastDriverAction, int numActions);
    bool operator==(Observation const& other) const;

    // std::vector<float> trackSensorData;
    float angle;
    float distToMiddle;
    float distToStart;
    float lastDriverAction;
    int numActions;

private:
    // static const int numSensors;
    //static const float sensorRange; 
    // static const std::vector<float> trackSensAngle;
    // static Sensors *trackSensors;
    // static const std::vector<float> sensorBins;
    static const int numAngleBins = 100001; // Must be an odd number!
    static const int numMiddleBins = 51; // Must be an odd number!
    static constexpr float startBinSize = 0.05f;
    static const std::vector<float> angleBins;
    static const std::vector<float> middleBins;
};

}

namespace std {
  template <> struct hash<pomdp::Observation>
  {
    size_t operator()(const pomdp::Observation & o) const
    {
        using boost::hash_value;
		using boost::hash_combine;
        size_t seed = 0;
        // boost::hash_combine(seed, hash_value(o.trackSensorData));
        hash_combine(seed, hash_value(o.angle));
        // boost::hash_combine(seed, hash_value(o.distToMiddle));
        hash_combine(seed, hash_value(o.distToStart));
        hash_combine(seed, hash_value(o.lastDriverAction));
        // hash_combine(seed, hash_value(o.numActions));
        return seed;
    }
  };
}

namespace pomdp {

typedef float Action;

class RewardCalculator 
{
public:
    static double reward(const tSituation& situation, const Action& action);
private:
    static double rewardPosition(const tSituation& situation);
    static double penaltyActionIntensity(const Action& action);
};

inline State::State() {}

inline 
State::State(const State& other) : situation{ other.situation }, car{ other.car }, modelState{ other.modelState }, actionsCount{ other.actionsCount } {}

inline 
State::State(tSituation& situation, tCar& car, DriverModelState& modelState, int actionsCount) : situation{ situation }, car{ car }, modelState{ modelState }, actionsCount{ actionsCount } {}

inline
bool State::isTerminal() {
    tCarElt* car = situation.cars[0];
    double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
	return absDistToMiddle >= TERMINAL_OFF_LANE_DIST;
}

inline
bool State::isTerminal(tSituation& situation) {
    tCarElt* car = situation.cars[0];
    double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
	return absDistToMiddle >= TERMINAL_OFF_LANE_DIST;
}

// const int Observation::numSensors = 5;
// const float Observation::sensorRange = 100;
// const std::vector<float> Observation::trackSensAngle = { -80, -40, 0, 40, 80 };
// Sensors *Observation::trackSensors = [](){
//     Sensors *s = new Sensors(numSensors);
//     for (int i = 0; i < numSensors; ++i) {
//         s->setSensor(i, trackSensAngle[i], sensorRange);
//     }
//     return s;
// }();
// const std::vector<float> Observation::sensorBins = {0.5, 1, 1.5, 2, 2.5, 3.5, 5, 7.5, 10, 15, 20, 25, 30, 50, 100}; // Up-to and including distance, beyond -> -1

inline
const std::vector<float> Observation::angleBins = [](){ // [-PI, ..., PI]
    std::vector<float> bins;
    bins.reserve(numAngleBins);
    int mult = -(numAngleBins / 2);
    for (int i = 0; i < numAngleBins; i++) {
            bins.push_back(mult * (PI / (numAngleBins / 2)));
            mult++;
    }
    return bins;
}();

inline
const std::vector<float> Observation::middleBins = [](){ // [neg. out of lane, -1, ..., +1 , pos. out of lane]
    std::vector<float> bins;
    bins.reserve(numMiddleBins);
    int numInLane = numMiddleBins - 2;
    bins.push_back(std::nextafter(-1.0f, -2.0f)); // Negative out-of-lane
    int mult = -(numInLane / 2);
    for (int i = 0; i < numInLane; i++) {
            bins.push_back(mult * (1.0f / (numInLane / 2))); // Within lane
            mult++;
    }
    bins.push_back(std::nextafter(1.0f, 2.0f)); // Positive out-of-lane
    return bins;
}();

inline
Observation::Observation(tSituation& situation, float lastDriverAction, int numActions) : lastDriverAction{ lastDriverAction }, numActions{ numActions }
{
    tCarElt *car = situation.cars[0];

    // Get track sensor output
    // set the value of track sensors only as long as the car is inside the track
    // if (distToMiddle <= 1.0 && distToMiddle >= -1.0) {
    //     trackSensors->sensors_update(car);
    //     trackSensorData.reserve(numSensors);
    //     for (int i = 0; i < numSensors; ++i) {
    //         float distance = trackSensors->getSensorOut(i);
    //         if (distance < 0) {
    //             distance = -1; // car behind track border
    //         } else {
    //             distance = Discretizer::search(sensorBins, 0, sensorBins.size() - 1, distance);
    //         }
    //         trackSensorData.push_back(distance);
    //     }
    // } else {
    //     for (int i = 0; i < numSensors; ++i) {
    //         trackSensorData.push_back(-1);
    //     }
    // }

    // Compute distance to middle
    // distToMiddle = 2 * car->_trkPos.toMiddle / (car->_trkPos.seg->width);
    // distToMiddle = utils::Discretizer::discretize(middleBins, distToMiddle);

    // Compute distance to start
    distToStart = car->_trkPos.seg->lgfromstart + 
        (car->_trkPos.seg->type == TR_STR ? car->_trkPos.toStart : car->_trkPos.toStart * car->_trkPos.seg->radius);
    distToStart = distToStart >= startBinSize ? std::round(distToStart / startBinSize) : 0;

    // Compute the car angle wrt. the track axis
    const float SC = 1.0;
    angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI
    angle -= SC * car->_trkPos.toMiddle / car->_trkPos.seg->width;
    angle = utils::Discretizer::discretize(angleBins, angle);
}

inline
bool Observation::operator==(Observation const& other) const
{
    // return trackSensors == other.trackSensors 
    //     && angle == other.angle 
    //     && distToMiddle == other.distToMiddle;
    // return angle == other.angle && distToMiddle == other.distToMiddle && distToStart == other.distToStart && lastDriverAction == other.lastDriverAction;
    //return angle == other.angle && distToMiddle == other.distToMiddle && distToStart == other.distToStart;
    return angle == other.angle && distToStart == other.distToStart && lastDriverAction == other.lastDriverAction;
}

inline
double RewardCalculator::reward(const tSituation& situation, const Action& action) {
    return rewardPosition(situation) - penaltyActionIntensity(action);
}

inline
double RewardCalculator::rewardPosition(const tSituation& situation) {
    tCarElt *car = situation.cars[0];
    double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
    return absDistToMiddle <= 1 ? REWARD_CENTER - REWARD_CENTER * absDistToMiddle : PENALTY_OFF_LANE;
}

inline
double RewardCalculator::penaltyActionIntensity(const Action& action) {
    // return pow(abs(a), PENALTY_INTENSITY_EXP);
    return 0;
}

}