#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <math.h>

using std::vector;

namespace pomdp
{
    /* POMCP */
    static const double THRESHOLD = 0.1;

    /* experiment constants */
    static const unsigned TARGET_RUNS = 1;
    static const float TARGET_SPEED = 13.9;         /* [m/s] 27.8 = ~100 kph */	
    static const double STEER_FREQ = 0.1;

    /* episode constants */
    static const unsigned TARGET_ACTIONS = 100;	

    /* grid search constants */
    static const bool SEARCH_DISCOUNT = true;

    /* race constants */
    static const float FULL_ACCEL_MARGIN = 1.0;     /* [m/s] */
    static const float SHIFT = 0.9;                 /* [-] (% of rpmredline) */
    static const float SHIFT_MARGIN = 4.0;          /* [m/s] */

    /* hyperparameter constants */
    static const vector<vector<float>> ACTION_SCENARIOS{ {-1, -0.5, 0, 0.5, 1}, {-1, -0.5, -0.25, -0.1, 0, 0.1, 0.25, 0.5, 1} };
    static const vector<int> BIN_SCNEARIOS{ 101, 1001, 10001 };
    static const vector<double> PLANNING_TIME_SCENARIOS{ 0.1, 0.5, 1.0 };
    static const vector<double> DISCOUNT_SCENARIOS{ THRESHOLD, 
												    pow(THRESHOLD, 1/5), // TODO: Verify that this works correctly!
												    pow(THRESHOLD, 1/10), 
												    pow(THRESHOLD, 1/15), 
												    pow(THRESHOLD, 1/20) }; // 1, 5, 10, 15, 20 actions discount horizon
}

#endif