#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <math.h>
#include <vector>
#include <string>

using std::vector;
using std::string;

namespace pomdp
{
    /* POMCP */
    typedef float Action;
    static const double THRESHOLD = 0.1;
    static const unsigned int PARTICLES = 1000;
    static const bool PARTICLE_REINV = true;
    static const double TRANSFER_QUOTA = 1.0/16;
    static const bool PARTICLE_RESAMP = false;
    static const double RESAMP_QUOTA = 1.0;

    /* experiment constants */
    static const unsigned TARGET_RUNS = 20;
    static const float TARGET_SPEED = 80 * 1000 / 60 / 60;         /* [m/s] ~80 kph */	
    static const double STEER_ACTION_FREQ = 0.1;
    static constexpr float STEER_LOOKAHEAD = 5;	// [m]
    static const bool TWO_LANE_TRACK = true;
    static const bool AUTONOMOUS = false; // No driver, just the planner

    /* episode constants */
    static const unsigned TARGET_ACTIONS = 600;	

    /* grid search constants */
    static const string INITIAL_AGENT_SCENARIO = "planner"; // "planner" -> "driver" -> "optimal"
    static const bool SEARCH_DISCOUNT = false;
    static const double DEFAULT_DISCOUNT = THRESHOLD / 2; // 5 Actions

    /* race constants */
    static const float FULL_ACCEL_MARGIN = 1.0;     /* [m/s] */
    static const float SHIFT = 0.9;                 /* [-] (% of rpmredline) */
    static const float SHIFT_MARGIN = 4.0;          /* [m/s] */

    /* hyperparameter constants */
    static const float REWARD_CENTER = 1;
    static const float PENALTY_OFF_LANE = -10.0;
    static const float PENALTY_INTENSITY_EXP = 2;

    static const float START_BIN_SIZE = 0.05;
    static const float TERMINAL_OFF_LANE_DIST = 1.05;

    static const vector<vector<float>> ACTION_SCENARIOS{ {-2, -1, -0.75, -0.5, -0.25, -0.15, -0.1, 0, 0.1, 0.15, 0.25, 0.5, 0.75, 1, 2} }; // {-2, -1, -0.5, 0, 0.5, 1, 2}, {-2, -1, -0.75, -0.5, -0.25, -0.15, -0.1, 0, 0.1, 0.15, 0.25, 0.5, 0.75, 1, 2}, {-2, -1.75, -1.5, -1.25, -1.15, -1.1, -1, -0.75, -0.5, -0.25, -0.15 -0.1, 0, 0.1, 0.15, 0.25, 0.5, 0.75, 1, 1.1, 1.15, 1.25, 1.5, 1.75, 2}
    static const vector<int> BIN_SCNEARIOS{ 1001 }; // Must be an odd number
    static const vector<unsigned> NUM_SIMS_SCENARIOS{ 1000 }; // 10, 100, 200, 300, 400, 500, 750, 1000, 1500, 2500, 5000, 7500, 10000
    static const vector<double> DISCOUNT_SCENARIOS
    { 
        THRESHOLD / 2, // Discount horizon:  1
        pow(THRESHOLD, (1.0/4)),         //  5
        pow(THRESHOLD, (1.0/9)),         // 10
        pow(THRESHOLD, (1.0/14)),        // 15
        pow(THRESHOLD, (1.0/19)),        // 20 
        pow(THRESHOLD, (1.0/29)),        // 30
        pow(THRESHOLD, (1.0/39)),        // 40
        pow(THRESHOLD, (1.0/49)),        // 50
        pow(THRESHOLD, (1.0/59))         // 60
    };
    static const vector<double> EXP_CONST_SCENARIOS{ 0.5 }; // 0, 0.25, 0.5, 0.75, 1, 2, 3, 4, 5, 10, 15, 25

    /* driver model constants */
    static const bool INITIAL_ATTENTIVE = false;
    static const bool DRIVER_OVER_CORRECT = true;
    static const double DRIVER_COR_FACTOR_MIN = 0.10;
    static const double DRIVER_COR_FACTOR_MAX = 0.25;
    static const bool DRIVER_DISCRETE_ACTIONS = true;
    static const bool DRIVER_ACTION_NOISE = false;
    static const double DRIVER_NOISE_DIST_MAX = 0.15;
    static const double DRIVER_NOISE_ATT_MAX = 0.05;
    static const unsigned MIN_ATTENTIVE_ACTIONS = (float) 1 / STEER_ACTION_FREQ; // #Actions for one second of simulated time
    static const unsigned MAX_ATTENTIVE_ACTIONS = (float) 5 / STEER_ACTION_FREQ; // #Actions for five seconds of simulated time
    static const unsigned MIN_DISTRACTED_ACTIONS = (float) 1 / STEER_ACTION_FREQ; // #Actions for one second of simulated time
    static const unsigned MAX_DISTRACTED_ACTIONS = (float) 5 / STEER_ACTION_FREQ; // #Actions for five seconds of simulated time
}

#endif