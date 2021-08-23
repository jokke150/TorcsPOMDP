#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <math.h>

using std::vector;

namespace pomdp
{
    /* POMCP */
    typedef float Action;
    static const double THRESHOLD = 0.1;
    static const double RESAMPLING_TIME = 1.0;
    static const double EXPLORATION_CTE = 100;
    static const unsigned int PARTICLES = 100000;


    /* experiment constants */
    static const unsigned TARGET_RUNS = 1;
    static const float TARGET_SPEED = 27.8;         /* [m/s] 27.8 = ~100 kph */	
    static const double STEER_ACTION_FREQ = 0.1;

    /* episode constants */
    static const unsigned TARGET_ACTIONS = 100;	

    /* grid search constants */
    static const bool SEARCH_DISCOUNT = false;

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

    static const vector<vector<float>> ACTION_SCENARIOS{ {-1, -0.5, -0.25, -0.1, 0, 0.1, 0.25, 0.5, 1} }; // {-1, -0.5, 0, 0.5, 1}
    static const vector<int> BIN_SCNEARIOS{ 1001, 10001 }; // Must be an odd number
    static const vector<double> PLANNING_TIME_SCENARIOS{ 1.0 };
    static const vector<double> DISCOUNT_SCENARIOS{ THRESHOLD, 
                                                    pow(THRESHOLD, (1.0/5)), 
                                                    pow(THRESHOLD, (1.0/10)), 
												    pow(THRESHOLD, (1.0/15)), 
												    pow(THRESHOLD, (1.0/20)) }; 

    /* driver model constants */
    // static const unsigned MIN_ATTENTIVE_ACTIONS = (float) 1 / STEER_ACTION_FREQ; // #Actions for one second of simulated time
    // static const unsigned MAX_ATTENTIVE_ACTIONS = (float) 5 / STEER_ACTION_FREQ; // #Actions for five seconds of simulated time
    // static const unsigned MIN_DISTRACTED_ACTIONS = (float) 1 / STEER_ACTION_FREQ; // #Actions for one second of simulated time
    // static const unsigned MAX_DISTRACTED_ACTIONS = (float) 5 / STEER_ACTION_FREQ; // #Actions for five seconds of simulated time

    static const unsigned MIN_ATTENTIVE_ACTIONS = 1; // #Actions for one second of simulated time
    static const unsigned MAX_ATTENTIVE_ACTIONS = 5; // #Actions for five seconds of simulated time
    static const unsigned MIN_DISTRACTED_ACTIONS = 1; // #Actions for one second of simulated time
    static const unsigned MAX_DISTRACTED_ACTIONS = 5; // #Actions for five seconds of simulated time

}

#endif