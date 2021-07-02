#include "TorcsPomdp.hpp"
#include "Discretizer.hpp"

namespace pomdp
{

State::State() {}
State::State(const State& other) { torcsState = tSituation{other.torcsState}; modelState = DriverModelState{other.modelState}; }
State::State(tSituation torcsState, DriverModelState modelState) : torcsState{torcsState}, modelState{modelState} {}

bool State::isTerminal() {
    tCarElt* car = torcsState.cars[0];
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

const int Observation::numAngleBins = 51; // Must be an odd number!
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

const int Observation::numMiddleBins = 51; // Must be an odd number!
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

const float Observation::startBinSize = 0.05f;

Observation::Observation(State& s, float lastDriverAction) : lastDriverAction{ lastDriverAction }
{
    tCarElt *car = s.torcsState.cars[0];

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
    // distToStart = car->_trkPos.seg->lgfromstart +
	// (car->_trkPos.seg->type == TR_STR ? car->_trkPos.toStart : car->_trkPos.toStart * car->_trkPos.seg->radius);
    // distToStart = std::round(distToStart / startBinSize);

    // Compute the car angle wrt. the track axis
    const float SC = 1.0;
    angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI
    angle -= SC * car->_trkPos.toMiddle / car->_trkPos.seg->width;
    angle = utils::Discretizer::discretize(angleBins, angle);
}

bool Observation::operator==(Observation const& other) const
{
    // return trackSensors == other.trackSensors 
    //     && angle == other.angle 
    //     && distToMiddle == other.distToMiddle;
    // return angle == other.angle && distToMiddle == other.distToMiddle && distToStart == other.distToStart && lastDriverAction == other.lastDriverAction;
    //return angle == other.angle && distToMiddle == other.distToMiddle && distToStart == other.distToStart;
    return angle == other.angle;
}

double RewardCalculator::reward(const State& s, const Action& a) {
    return rewardPosition(s) - penaltyActionIntensity(a);
}

double RewardCalculator::rewardPosition(const State& s) {
    tCarElt *car = s.torcsState.cars[0];
    double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
    return absDistToMiddle <= 1 ? REWARD_CENTER - REWARD_CENTER * absDistToMiddle : PENALTY_OFF_LANE;
}

double RewardCalculator::penaltyActionIntensity(const Action& a) {
    return pow(abs(a), PENALTY_INTENSITY_EXP);
}

}