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
    static const double PREF_ACT_SD = 0.4; // 20% Chance of actions <=-0.5 and >=0.5
    static const double PREF_ACT_REWARD_BASE = 0.9;

    /* experiment constants */
    static const float TERMINAL_OFF_LANE_DIST = 1.05; // When to terminate episode
    static const float TARGET_SPEED = 80 * 1000 / 60 / 60; // [m/s] ~80 kph
    static const double STEER_ACTION_FREQ = 0.1; // how often is the driver called [s]
    static constexpr float STEER_LOOKAHEAD = 5;	// [m]
    static const bool TWO_LANE_TRACK = true; // divide the track width by two, if two lane
    static const bool AUTONOMOUS = false; // No driver, just the planner

    /* race constants */
    static const float FULL_ACCEL_MARGIN = 1.0;     /* [m/s] */
    static const float SHIFT = 0.9;                 /* [-] (% of rpmredline) */
    static const float SHIFT_MARGIN = 4.0;          /* [m/s] */

    /* discretization constants */
    static const unsigned NUM_BINS = 1001; // Must be an odd number
    static const float START_BIN_SIZE = 0.05;

    /* driver model constants */
    static const double DRIVER_COR_FACTOR_MIN = 0.10;
    static const double DRIVER_COR_FACTOR_MAX = 0.25;
    static const bool DRIVER_DISCRETE_ACTIONS = true;
    static const double DRIVER_NOISE_DIST_MAX = 0.15;
    static const double DRIVER_NOISE_ATT_MAX = 0.05;
    static const unsigned MIN_ATTENTIVE_ACTIONS = (float) 1 / STEER_ACTION_FREQ; // #Actions for one second of simulated time
    static const unsigned MAX_ATTENTIVE_ACTIONS = (float) 5 / STEER_ACTION_FREQ; // #Actions for five seconds of simulated time
    static const unsigned MIN_DISTRACTED_ACTIONS = (float) 1 / STEER_ACTION_FREQ; // #Actions for one second of simulated time
    static const unsigned MAX_DISTRACTED_ACTIONS = (float) 5 / STEER_ACTION_FREQ; // #Actions for five seconds of simulated time
}

#endif