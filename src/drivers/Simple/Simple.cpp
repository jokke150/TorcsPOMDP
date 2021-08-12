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

// Experiment variables
static const int targetRuns = 1;
static unsigned int runs = 0;
static double totalReward = 0;

// Episode variables
static const int targetActions = 100;
unsigned long actionsCount;
TorcsSimulator* simulator;
PomcpPlanner<State, Observation, Action, pomcp::VectorBelief<State>>* planner;
DriverModel* driverModel;
double reward;
double discount;

// Grid search variables
static std::string agentScenario = "planner"; // "planner", "driver", "optimal"
static int binsScenarioIdx = 0;
static int actionsScenarioIdx = 0;
static int planningTimeScenarioIdx = 0;
static const bool DISCOUNT_SEARCH = true;
static int discountScenarioIdx = 0;
static std::vector<Action> actions;
static std::vector<Action> driverActions;
static double planningTime;
static int numBins;

// Last actions
unsigned int lastActIdx;
float lastDriverAction;
float lastOptimalAction;
float lastCombinedAction;

// Target speed
const float FULL_ACCEL_MARGIN = 1.0;
const float SHIFT = 0.9;         /* [-] (% of rpmredline) */
const float SHIFT_MARGIN = 4.0;  /* [m/s] */
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

static int getGear(tCarElt *car);
static float getAccel(tCarElt* car);

static void gridSearch();
static void plan(tCarElt* car, tSituation *s);
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

    // Grid search for hyper parameters
    if (DISCOUNT_SEARCH) {
        binsScenarioIdx = 2;
        actionsScenarioIdx = 1;
        planningTimeScenarioIdx = 2;
    }
    if (runs == targetRuns) {
        runs = 0;
        if (!DISCOUNT_SEARCH) {
            if (actionsScenarioIdx == actionsScenarios.size() - 1) {
                actionsScenarioIdx = 0;
                if (agentScenario == "planner") {
                    if (binsScenarioIdx == binsScenarios.size() - 1) {
                        binsScenarioIdx = 0;
                        if (planningTimeScenarioIdx == planningTimesScenarios.size() - 1) {
                            agentScenario = "driver";
                        } else {
                            planningTimeScenarioIdx++;
                        }
                    } else {
                        binsScenarioIdx++;
                    }
                } else if (agentScenario == "driver") {
                    agentScenario = "optimal";
                } else {
                    // Experiment is finished
                    car->END = true;
                    return;
                }
            } else{
                actionsScenarioIdx++;
            }
        } else {
            if (discountScenarioIdx < discountScenarios.size() - 1) {
                discountScenarioIdx++;
            } else {
                // Experiment is finished
                car->END = true;
                return;
            }
        }
    }

    if (runs == 0) {
        actions = actionsScenarios[actionsScenarioIdx];
        // Driver can only steer half. Ultimately, the agent is in control.
        driverActions = agentScenario == "planner" ? std::vector<Action>(actions.begin() + 1, actions.end() - 1) : actions; 
        planningTime = planningTimesScenarios[planningTimeScenarioIdx];
        Observation::setAngleBins(binsScenarios[binsScenarioIdx]);
        Observation::setMiddleBins(binsScenarios[binsScenarioIdx]);
    }

    // CSV Writer
    ofs.open (agentScenario
                  + " a" + std::to_string(actionsScenarioIdx) 
                  + (agentScenario == "planner" ? " b" + std::to_string(binsScenarioIdx) : "") 
                  + (agentScenario == "planner" ? " p" + std::to_string(planningTimeScenarioIdx) : "") 
                  + (DISCOUNT_SEARCH ? " d" + std::to_string(discountScenarioIdx) : "")
                  + ".csv"
                  , std::ofstream::out | std::ofstream::app);
    if (runs == 0) {
        writer << std::vector<std::string>({"Run", "Count", "Cheat", "Terminal", "Size", "Depth", "Speed", "Angle", "Reward", "Gain", "From Start", 
                            "To Middle", "Distracted", "Time", "NumActions", "Optimal", "Combined", 
                            "Agent", "Driver" });
    }

    runs++;
} 

/* Drive during race. */
static void
drive(int index, tCarElt* car, tSituation *s, tRmInfo *ReInfo)
{
    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    // Actions that cannot be influenced by planner
    car->_gearCmd = getGear(car); // first gear
    car->_brakeCmd = 0;
    if (targetSpeed > car->_speed_x) {
        car->ctrl.accelCmd = getAccel(car);
    } else {
        car->ctrl.accelCmd = 0.0;
    }

    TorcsState torcsState{ *s };

    // We want to reach a certain initial speed before planning starts
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
        if (agentScenario == "planner") {
            simulator = new TorcsSimulator{ *s, *ReInfo, actions, driverActions, numBins, discountScenarios[discountScenarioIdx] };
            planner = new PomcpPlanner<State, Observation, Action, pomcp::VectorBelief<State>>{ *simulator, 
                planningTime, RESAMPLING_TIME, THRESHOLD, EXPLORATION_CTE, PARTICLES }; 
        }
        driverModel = new DriverModel(driverActions);  
    }

    // Planning
    if (actionsCount) {
        // Only update planner after first action
        
        Observation obs = Observation(*s, lastDriverAction, actionsCount, actions);

        bool cheat = false;
        unsigned depth, size;
        depth = size = 0;
        if (agentScenario == "planner") {
            planner->computeInfo(size,depth);
            cheat = planner->moveTo(lastActIdx, obs); // TODO: Actually, this determines if the planner will cheat for the next action, not the last one
        }

        // Calculate and sum up reward
        // discount *= simulator->getDiscount();
        // reward  += discount * RewardCalculator::reward(*s, actions[lastActIdx]);
        double rewardGain = RewardCalculator::reward(*s, actions[lastActIdx]);
        reward += rewardGain;

        double distToStart = car->_trkPos.seg->lgfromstart + 
            (car->_trkPos.seg->type == TR_STR ? car->_trkPos.toStart : car->_trkPos.toStart * car->_trkPos.seg->radius);
        double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
        bool isDistracted = driverModel->getState().isDistracted;
        unsigned numActions = driverModel->getState().numActions;

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
        
        bool isTerminal = State::isTerminal(*s);

        writer << std::make_tuple(runs, actionsCount, cheat ? "cheat" : "fair", isTerminal, size, depth, speed, torcsState.angle, reward, rewardGain, distToStart, 
                                  absDistToMiddle, isDistracted, s->currentTime, numActions, lastOptimalAction, lastCombinedAction,
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

    float driverAction = 0;
    int agentActionIdx = 0;
    float optimalAction = torcsState.angle / torcsState.steerLock;
    optimalAction = utils::Discretizer::discretize(actions, optimalAction);
    if (agentScenario != "optimal") {
        // Determine driver's action (discretized)
        driverModel->update(torcsState);
        driverAction = driverModel->getAction();

        float agentAction = 0;
        if (agentScenario == "planner") {
            agentActionIdx = planner->getAction();
            agentAction = actions[agentActionIdx];
        }

        // Combine steering actions
        car->_steerCmd = std::max(std::min(driverAction + agentAction, 1.0f), -1.0f);
    } else {
        car->_steerCmd = optimalAction;
    }
    
    actionsCount++;
    lastActIdx = agentActionIdx;
    lastOptimalAction = optimalAction;
    lastCombinedAction = car->_steerCmd;
    lastDriverAction = driverAction;
}

/* Compute gear */
int getGear(tCarElt *car)
{
    if (car->_gear <= 0) return 1;
    float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
    float omega = car->_enginerpmRedLine/gr_up;
    float wr = car->_wheelRadius(2);

    if (omega*wr*SHIFT < car->_speed_x) {
        return car->_gear + 1;
    } else {
        float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
        omega = car->_enginerpmRedLine/gr_down;
        if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) {
            return car->_gear - 1;
        }
    }
    return car->_gear;
}

static float
getAccel(tCarElt* car)
{
    // TODO: Add controller that keeps speed constant (preferably at 100 kph)
    float gr = car->_gearRatio[car->_gear + car->_gearOffset];
    float rm = car->_enginerpmRedLine;
    if (targetSpeed > car->_speed_x + FULL_ACCEL_MARGIN) {
        return 1.0;
    } else {
        // return targetSpeed / car->_wheelRadius(REAR_RGT) * gr / rm;
        return targetSpeed - car->_speed_x + 0.19;
    }
}

static void
plan(tCarElt* car, tSituation *s)
{
    // TODO
}

static void
gridSearch()
{
    // TODO
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
    if (agentScenario == "planner") {
        delete planner;
        delete simulator;
    }
    delete driverModel;
}

