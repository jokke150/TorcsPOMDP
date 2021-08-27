#include <math.h>
#include <algorithm>
#include <boost/functional/hash.hpp>
#include <tgf.h>
#include <raceman.h>
#include <carstruct.h>

#include "sensors.h"
#include "DriverModel.hpp"
#include "Constants.h"

using std::vector;

namespace pomdp
{

typedef struct State 
{   
    State();
    State(const State& other);
    State(const tSituation& situation, const tCar& car, const DriverModelState& modelState, int actionsCount);
    
    State& operator=(const State& other);
    
    bool isTerminal();
    static bool isTerminal(tSituation& situation);

    tSituation situation;
    tCar car;
    DriverModelState modelState;
    int actionsCount;
} State;

inline State::State() {}

inline 
State::State(const State& other) : modelState{ other.modelState }, actionsCount{ other.actionsCount } 
{
    situation = other.situation;
    car = tCar{ other.car, situation.cars[0] };
}

inline 
State::State(const tSituation& situation, const tCar& car, const DriverModelState& modelState, int actionsCount) : modelState{ modelState }, actionsCount{ actionsCount } 
{
    this->situation = situation;
    this->car = tCar{ car, this->situation.cars[0] };
}

inline
State& State::operator=(const State& other) 
{
    if (this == &other) return *this;
    situation = other.situation;
    car = tCar{ other.car, situation.cars[0] };
    modelState = other.modelState;
    actionsCount = other.actionsCount;
    return *this;
}

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

class Observation
{   
public:
    Observation() = default;
    Observation(tSituation& situation, float lastDriverAction, int numActions, vector<Action>& actions);
    bool operator==(Observation const& other) const;
    
    static void setAngleBins(int numAngleBins);
    static void setMiddleBins(int numMiddleBins);

    // vector<float> trackSensorData;
    float angle;
    float distToMiddle;
    float distToStart;
    float lastDriverAction;
    int numActions;
    float speed;

private:
    // static const int numSensors;
    //static const float sensorRange; 
    // static const vector<float> trackSensAngle;
    // static Sensors *trackSensors;
    // static const vector<float> sensorBins;
    static vector<float> angleBins; // [-PI, ..., PI]
    static vector<float> middleBins; // [neg. out of lane, -1, ..., +1 , pos. out of lane]
};

// const int Observation::numSensors = 5;
// const float Observation::sensorRange = 100;
// const vector<float> Observation::trackSensAngle = { -80, -40, 0, 40, 80 };
// Sensors *Observation::trackSensors = [](){
//     Sensors *s = new Sensors(numSensors);
//     for (int i = 0; i < numSensors; ++i) {
//         s->setSensor(i, trackSensAngle[i], sensorRange);
//     }
//     return s;
// }();
// const vector<float> Observation::sensorBins = {0.5, 1, 1.5, 2, 2.5, 3.5, 5, 7.5, 10, 15, 20, 25, 30, 50, 100}; // Up-to and including distance, beyond -> -1

// inline
// const vector<float> Observation::angleBins = [](){ // [-PI, ..., PI]
//     vector<float> bins;
    // bins.reserve(numAngleBins);
    // int mult = -(numAngleBins / 2);
    // for (int i = 0; i < numAngleBins; i++) {
    //         bins.push_back(mult * (PI / (numAngleBins / 2)));
    //         mult++;
    // }
    // return bins;
// }();

// inline
// const vector<float> Observation::middleBins = [](){ // [neg. out of lane, -1, ..., +1 , pos. out of lane]
//     vector<float> bins;
//     bins.reserve(numMiddleBins);
//     int numInLane = numMiddleBins - 2;
//     bins.push_back(std::nextafter(-1.0f, -2.0f)); // Negative out-of-lane
//     int mult = -(numInLane / 2);
//     for (int i = 0; i < numInLane; i++) {
//             bins.push_back(mult * (1.0f / (numInLane / 2))); // Within lane
//             mult++;
//     }
//     bins.push_back(std::nextafter(1.0f, 2.0f)); // Positive out-of-lane
//     return bins;
// }();

inline vector<float> Observation::angleBins = {};
inline vector<float> Observation::middleBins = {};

inline
Observation::Observation(tSituation& situation, float driverAction, int numActions, vector<Action>& actions) : 
    numActions{ numActions }
{
    tCarElt *car = situation.cars[0];

    // Get track sensor output
    // set the value of track sensors only as long as the car is inside the track
    // if (distToMiddle <= 1.0 && distToMiddle >= -1.0) {
    //     trackSensors->sensors_update(car);WS
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

    // Discretize last driver action
    lastDriverAction = utils::Discretizer::discretize(actions, driverAction);

    // Compute distance to middle
    distToMiddle = 2 * car->_trkPos.toMiddle / (car->_trkPos.seg->width);
    distToMiddle = utils::Discretizer::discretize(middleBins, distToMiddle);

    // Compute distance to start
    distToStart = car->_trkPos.seg->lgfromstart + 
        (car->_trkPos.seg->type == TR_STR ? car->_trkPos.toStart : car->_trkPos.toStart * car->_trkPos.seg->radius);
    distToStart = distToStart >= START_BIN_SIZE ? std::round(distToStart / START_BIN_SIZE) : 0;

    // Compute the car angle wrt. the track axis
    const float SC = 1.0;
    angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI
    angle -= SC * car->_trkPos.toMiddle / car->_trkPos.seg->width;
    angle = utils::Discretizer::discretize(Observation::angleBins, angle);

    speed = car->_speed_x;
}

inline
bool Observation::operator==(Observation const& other) const
{
    return angle == other.angle && 
           distToMiddle == distToMiddle && 
           lastDriverAction == other.lastDriverAction;
}

inline
void Observation::setAngleBins(int numAngleBins) {
    angleBins.clear();
    angleBins.reserve(numAngleBins);
    int mult = -(numAngleBins / 2);
    for (int i = 0; i < numAngleBins; i++) {
        angleBins.push_back(mult * (PI / (numAngleBins / 2)));
        mult++;
    }
}

inline
void Observation::setMiddleBins(int numMiddleBins) {
    middleBins.clear();
    middleBins.reserve(numMiddleBins);
    int numInLane = numMiddleBins - 2;
    middleBins.push_back(std::nextafter(-1.0f, -2.0f)); // Negative out-of-lane
    int mult = -(numInLane / 2);
    for (int i = 0; i < numInLane; i++) {
        middleBins.push_back(mult * (1.0f / (numInLane / 2))); // Within lane
        mult++;
    }
    middleBins.push_back(std::nextafter(1.0f, 2.0f)); // Positive out-of-lane
}

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
        hash_combine(seed, hash_value(o.distToMiddle));
        // hash_combine(seed, hash_value(o.distToStart));
        hash_combine(seed, hash_value(o.lastDriverAction));
        // hash_combine(seed, hash_value(o.speed));
        // hash_combine(seed, hash_value(o.numActions));
        return seed;
    }
  };
}

namespace pomdp {

class RewardCalculator 
{
public:
    static double reward(const tSituation& nextState, const Action& action);
private:
    static double rewardPosition(const tSituation& situation);
    static double penaltyActionIntensity(const tSituation& situation, const Action& action);
    static double rewardAngle(const tSituation& situation);
    static double rewardAngleAndPosition(const tSituation& situation);
};

inline
double RewardCalculator::reward(const tSituation& nextState, const Action& action) 
{
    return rewardAngleAndPosition(nextState);
}

/** 
 * From: Design of a Reinforcement Learning-Based Lane Keeping Planning Agent for Automated Vehicles
 */
inline
double RewardCalculator::rewardAngleAndPosition(const tSituation& situation) 
{
    tCarElt *car = situation.cars[0];
    double distToMiddle = 2 * car->_trkPos.toMiddle / (car->_trkPos.seg->width);
    float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI
    angle -= car->_trkPos.toMiddle / car->_trkPos.seg->width;
    angle = angle / PI; // normalize to [-1,1]
    double reward = cos(angle + abs(distToMiddle));
    return reward > 0 ? reward : 0;
}

inline
double RewardCalculator::rewardPosition(const tSituation& situation) 
{
    tCarElt *car = situation.cars[0];
    double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
    double reward = absDistToMiddle <= 1 ? REWARD_CENTER * std::pow(0.01, absDistToMiddle) : PENALTY_OFF_LANE;
    return reward;
}

inline
double RewardCalculator::penaltyActionIntensity(const tSituation& situation, const Action& action) 
{
    tCarElt *car = situation.cars[0];
    double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
    double penalty = (1 - std::min(absDistToMiddle, 1.0)) * pow(abs(action), PENALTY_INTENSITY_EXP);
    return penalty;
}

inline
double RewardCalculator::rewardAngle(const tSituation& situation) 
{
    tCarElt *car = situation.cars[0];
    double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
    const float SC = 1.0;
    float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI
    angle -= SC * car->_trkPos.toMiddle / car->_trkPos.seg->width;
    float absAngle = abs(angle); 
    absAngle /= PI; // normalize to [0, 1]
    double reward = absDistToMiddle <= 1 ? REWARD_CENTER * std::pow(0.01, absAngle) : PENALTY_OFF_LANE;
    return reward;
}

}