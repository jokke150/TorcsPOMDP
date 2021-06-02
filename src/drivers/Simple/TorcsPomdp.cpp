#include "TorcsPomdp.hpp"

namespace pomdp
{

const int Observation::numSensors = 5;
const float Observation::sensorRange = 100;
const std::vector<float> Observation::trackSensAngle = { -80, -40, 0, 40, 80 };
Sensors *Observation::trackSensors = [](){
    Sensors *s = new Sensors(numSensors);
    for (int i = 0; i < numSensors; ++i) {
        s->setSensor(i, trackSensAngle[i], sensorRange);
    }
    return s;
}();
const std::vector<float> Observation::sensorBins = {0.5, 1, 1.5, 2, 2.5, 3.5, 5, 7.5, 10, 15, 20, 25, 30, 50, 100}; // Up-to and including distance, beyond -> -1
const int Observation::numAngleBins = 10;
const std::vector<float> Observation::angleBins = [](){ // Up-to and including angle, from -PI to PI 
    std::vector<float> bins;
    int mult = -(numAngleBins / 2) + 1;
    for (float& bin : bins) {
            bin = mult * (PI / (numAngleBins / 2));
            mult++;
    }
    return bins;
}();
const std::vector<float> Observation::middleBins = {-1.0, -0.7, -0.5, -0.3, -0.15, -0.05, 0, 0.05, 0.15, 0.3, 0.5, 0.7, 1.0}; // Up-to and including distance, beyond -> -1

Observation::Observation(State& s) 
{
    tCarElt *car = s.torcsState.cars[0];

    // Get track sensor output
    // update the value of track sensors only as long as the car is inside the track
    if (distToMiddle <= 1.0 && distToMiddle >= -1.0) {
        trackSensors->sensors_update(car);
        for (int i = 0; i < numSensors; ++i) {
            trackSensorData.at(i) = trackSensors->getSensorOutDiscrete(i, sensorBins, sensorBins.size());
        }
    } else {
        for (int i = 0; i < numSensors; ++i) {
            trackSensorData.at(i) = -1;
        }
    }

    // Compute distance to middle
    distToMiddle = 2*car->_trkPos.toMiddle/(car->_trkPos.seg->width);
    distToMiddle = Discretizer::search(middleBins, 0, middleBins.size() - 1, distToMiddle);

    // Compute the car angle wrt. the track axis
    angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI
    angle = Discretizer::search(angleBins, 0, angleBins.size() - 1, angle);
}

bool Observation::operator==(Observation const& other) const
{
    return trackSensors == other.trackSensors 
        && angle == other.angle 
        && distToMiddle == other.distToMiddle;
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

int Discretizer::search(const std::vector<float>& vec, int start_idx, int end_idx, float search_val) {
	if( start_idx == end_idx )
		return vec[start_idx] >= search_val ? start_idx : -1;

	int mid_idx = start_idx + (end_idx - start_idx) / 2;

	if(search_val <= vec[mid_idx])
		return search(vec, start_idx, mid_idx, search_val);

	return search(vec, mid_idx+1, end_idx, search_val);
}

}