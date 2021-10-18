#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <math.h>
#include <vector>
#include <string>

using std::vector;
using std::string;

namespace pomdp
{
    /* experiment constants */
    static const float TERMINAL_OFF_LANE_DIST = 1.05; // When to terminate episode
    static const float TARGET_SPEED = 80 * 1000 / 60 / 60; // [m/s] ~80 kph
    static const double STEER_ACTION_FREQ = 0.1; // how often is the driver called [s]
    static constexpr float STEER_LOOKAHEAD = 5;	// [m]
    static const bool TWO_LANE_TRACK = true; // divide the track width by two, if two lane

    /* race constants */
    static const float FULL_ACCEL_MARGIN = 1.0;     /* [m/s] */
    static const float SHIFT = 0.9;                 /* [-] (% of rpmredline) */
    static const float SHIFT_MARGIN = 4.0;          /* [m/s] */
}

#endif