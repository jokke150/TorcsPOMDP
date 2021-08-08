/***************************************************************************

    file                 : Simple.cpp
    created              : Do 22. Apr 11:54:37 CEST 2021
    copyright            : (C) 2002 Jokke Jansen

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>
#include <stdexcept>
#include <iostream>
#include <algorithm>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

#include "TorcsSimulator.hpp"
#include "Pomcp.hpp"

#include <csv.hpp>

using namespace pomcp;
using namespace pomdp;
using namespace csv;

static tTrack *curTrack;

// Grid search variables
static int binsScenarioIdx = 0;
static int actionsScenarioIdx = 0;
static int planningTimeScenarioIdx = 0;
static std::vector<Action> actions;
static std::vector<Action> driverActions;
static double planningTime;
static int numBins;

// Experiment variables
static const int targetRuns = 100;
static unsigned int runs = 0;
static double totalReward = 0;

// Episode variables
static const int targetActions = 1200;
unsigned long actionsCount;
TorcsSimulator* simulator;
PomcpPlanner<State, Observation, Action, pomcp::VectorBelief<State>>* planner;
DriverModel* driverModel;
double reward;
double discount;

// Last actions
unsigned int lastActIdx;
float lastDriverAction;
float lastOptimalAction;
float lastCombinedAction;

// Target speed
static const float targetSpeed = 17.4;
static const int goalCallsTargetSpeed = 10;
int numCallsTargetSpeed;
bool isTargetSpeedReached;

// Persistence
static std::ofstream ofs;
static auto writer = make_csv_writer_buffered(ofs);

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s, tRmInfo *ReInfo); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 

static void restart(tCarElt* car); 


/* 
 * Module entry point  
 */ 
extern "C" int 
Simple(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("Simple");		/* name of the module (short) */
    modInfo->desc    = strdup("");	/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    curTrack = track;
    *carParmHandle = NULL; 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
    actionsCount = 0;
    reward = 0;
    discount = 1.0;
    numCallsTargetSpeed = 0;
    isTargetSpeedReached = false;

    // Grid search
    if (runs == targetRuns) {
        runs = 0;
        if (binsScenarioIdx == binsScenarios.size() - 1) {
            if (actionsScenarioIdx == actionsScenarios.size() - 1) {
                if (planningTimeScenarioIdx == planningTimesScenarios.size() - 1) {
                    // Experiment is finished!
                    car->END = true;
                    return;
                } else {
                    binsScenarioIdx = 0;
                    actionsScenarioIdx = 0;
                    planningTimeScenarioIdx++;
                }
            } else{
                binsScenarioIdx = 0;
                actionsScenarioIdx++;
            }
        } else {
            binsScenarioIdx++;
        }
    }
    if (runs == 0) {
        actions = actionsScenarios[actionsScenarioIdx];
        driverActions = std::vector<Action>(actions.begin() + 1, actions.end() - 1); // Driver can only steer half. Ultimately, the agent is in control.
        planningTime = planningTimesScenarios[planningTimeScenarioIdx];
        Observation::setAngleBins(binsScenarios[binsScenarioIdx]);
        Observation::setMiddleBins(binsScenarios[binsScenarioIdx]);
    }

    // CSV Writer
    ofs.open ("b" + std::to_string(binsScenarioIdx) + " a"  + std::to_string(actionsScenarioIdx) + " p" + std::to_string(planningTimeScenarioIdx) + ".csv", std::ofstream::out | std::ofstream::app);
    if (runs == 0) {
        writer << std::vector<std::string>({"Run", "Count", "Cheat", "Terminal" "Size", "Depth", "Speed", "Angle", "Reward", "Gain" "From Start", 
                            "To Middle", "Distracted", "Time", "Duration", "Optimal", "Combined", 
                            "Agent", "Driver" });
    }
    set_decimal_places(10);

    runs++;
} 

/* Drive during race. */
static void
drive(int index, tCarElt* car, tSituation *s, tRmInfo *ReInfo)
{
    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    // Constant actions (Cannot be influenced by planner)
    car->_accelCmd = 0.3; // 30% accelerator pedal 
    car->_brakeCmd = 0.0; // no brakes 
    car->_gearCmd = 1; // first gear
    car->_clutchCmd = 0.0; // no clutch

    TorcsState torcsState{ *s };

    // We want a constant target speed over a number of calls before starting the planning
    // TODO: Add controller that keeps speed constant (preferably at 100 kph)
    float speed = car->pub.speed;
    if (!isTargetSpeedReached) {
        if ((int) (speed * 10 + .05) == (int) (targetSpeed * 10 + .05)) {
            numCallsTargetSpeed++;
        } else {
            numCallsTargetSpeed = 0;
        }
        if (numCallsTargetSpeed < goalCallsTargetSpeed) {
            car->_steerCmd = torcsState.angle / torcsState.steerLock; // automatically steer to middle
            return;
        } else {
            isTargetSpeedReached = true;
        }
    }

    if (!actionsCount) {
        simulator = new TorcsSimulator{ *s, *ReInfo, actions, driverActions, numBins };
        planner = new PomcpPlanner<State, Observation, Action, pomcp::VectorBelief<State>>{ *simulator, 
            planningTime, RESAMPLING_TIME, THRESHOLD, EXPLORATION_CTE, PARTICLES };   
        driverModel = new DriverModel(s->currentTime, driverActions);
    }

    // Planning
    if (actionsCount) {
        // Only update planner after first action
        unsigned depth, size;

        depth = size = 0;
        planner->computeInfo(size,depth);
        Observation obs = Observation(*s, lastDriverAction, actionsCount, actions);

        // Calculate and sum up reward
        // discount *= simulator->getDiscount();
        // reward  += discount * RewardCalculator::reward(*s, actions[lastActIdx]);
        double rewardGain = RewardCalculator::reward(*s, actions[lastActIdx]);
        reward += rewardGain;

        const float SC = 1.0;
        float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
        NORM_PI_PI(angle); // normalize the angle between -PI and + PI
        angle -= SC * car->_trkPos.toMiddle / car->_trkPos.seg->width;
        double distToStart = car->_trkPos.seg->lgfromstart + 
            (car->_trkPos.seg->type == TR_STR ? car->_trkPos.toStart : car->_trkPos.toStart * car->_trkPos.seg->radius);
        double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
        bool isDistracted = driverModel->getState().isDistracted;
        double timeEpisodeEnd = driverModel->getState().timeEpisodeEnd;

        // std::cout<<"__________________________________"<<std::endl;
        // std::cout<<"Count: "<<actionsCount<<std::endl;
        // std::cout<<"Size: "<<size<<std::endl;
		// std::cout<<"Depth: "<<depth<<std::endl;
        // std::cout<<"Speed: "<<speed<<std::endl;
        // std::cout<<"Angle: "<<angle<<std::endl;
        // std::cout<<"Reward: "<<reward<<std::endl;
        // std::cout<<"From Start: "<<distToStart<<std::endl;
        // std::cout<<"To Middle: "<<absDistToMiddle<<std::endl;
        // std::cout<<"Driver distracted: "<<isDistracted<<std::endl;
        // std::cout<<"Current time: "<<s->currentTime<<std::endl;
        // std::cout<<"Driver state duration: "<<timeEpisodeEnd<<std::endl;
        // std::cout<<"Optimal: "<<lastOptimalAction<<std::endl;
        // std::cout<<"Combined: "<<lastCombinedAction<<std::endl;
        // std::cout<<"Agent: "<<actions[lastActIdx]<<std::endl;
        // std::cout<<"Driver: "<<lastDriverAction<<std::endl; 

        bool cheat = planner->moveTo(lastActIdx, obs);
        bool isTerminal = State::isTerminal(*s);

        writer << std::make_tuple(runs, actionsCount, cheat, isTerminal, size, depth, speed, angle, reward, rewardGain, distToStart, 
                                  absDistToMiddle, isDistracted, s->currentTime, timeEpisodeEnd, lastOptimalAction, lastCombinedAction,
                                  actions[lastActIdx], lastDriverAction);

        // Restart race and start next run if terminal state is reached
        if (isTerminal) {
            std::cout<<"Terminal state reached after "<<actionsCount<<" actions."<<std::endl;
            return restart(car);
        }

        // Restart race and start next run if target number of actions is reached
        if (actionsCount == targetActions) {
            std::cout<<"Episode finished after "<<actionsCount<<" actions."<<std::endl;
            return restart(car);
        }
    }

    int agentActionIdx = planner->getAction();
    float agentAction = actions[agentActionIdx];

    // Determine driver's action (discretized)
    driverModel->update(torcsState);
    float driverAction = utils::Discretizer::discretize(driverActions, driverModel->getAction());

    // Combine steering actions
    car->_steerCmd = std::max(std::min(driverAction + agentAction, 1.0f), -1.0f);

    actionsCount++;
    lastActIdx = agentActionIdx;
    lastOptimalAction = torcsState.angle / torcsState.steerLock;
    lastCombinedAction = car->_steerCmd;
    lastDriverAction = driverAction;
}

static void
restart(tCarElt* car)
{
    totalReward += reward;
    double avgReward = totalReward / runs;
    std::cout<<"Restarting"<<std::endl;
    std::cout<<"Average reward after "<<runs<<" runs: "<<avgReward<<std::endl;

    writer.flush();
    ofs.close();

    car->RESTART = true;
}


/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    delete planner;
    delete simulator;
    delete driverModel;
}

