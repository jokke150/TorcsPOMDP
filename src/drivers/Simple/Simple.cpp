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

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

#include "TorcsSimulator.hpp"
#include "Pomcp.hpp"

using namespace pomcp;
using namespace pomdp;

static tTrack *curTrack;

// POMCP
static TorcsSimulator* simulator = nullptr;
static PomcpPlanner<State, Observation, Action, pomcp::VectorBelief<State>>* planner = nullptr;

static DriverModel* driverModel = nullptr;

static std::vector<Action> actions{ -1, -0.6, -0.2, -0.1, -0.05, 0, 0.05, 0.1, 0.2, 0.6, 1 };
static unsigned int runs = 0;
static unsigned long actionsCount;
static unsigned int lastActIdx;
static float lastDriverAction;
static float lastOptimalAction;
static double reward; 
static double discount = 1.0;

static int numCallsTargetSpeed;
static bool isTargetSpeedReached = false;
static const float targetSpeed = 17.4;
static const int goalCallsTargetSpeed = 10;

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
    numCallsTargetSpeed = 0;
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

    if (!simulator) {
        simulator = new TorcsSimulator{ actions, *s, *ReInfo };
        planner = new PomcpPlanner<State, Observation, Action, pomcp::VectorBelief<State>>{ *simulator, PLANNING_TIME, RESAMPLING_TIME, THRESHOLD, EXPLORATION_CTE, PARTICLES };   
        driverModel = new DriverModel(s->currentTime);
    }

    // Restart race and start next run if terminal state is reached
    if (State::isTerminal(*s)) {
        std::cout<<"Terminal state reached."<<std::endl;
        restart(car);
    }

    // Planning
    if (actionsCount) {
        // Only update planner after first action
        unsigned depth,size;

        depth = size = 0;
        // planner->computeInfo(size,depth);
        Observation obs = Observation(*s, lastDriverAction, actionsCount);

        std::cout<<"__________________________________"<<std::endl;
        std::cout<<"Count: "<<actionsCount<<std::endl;
        std::cout<<"Size: "<<size<<std::endl;
		std::cout<<"Depth: "<<depth<<std::endl;
        std::cout<<"Angle: "<<obs.angle<<std::endl;
        std::cout<<"Reward: "<<reward<<std::endl;
        double absDistToMiddle = abs(2*car->_trkPos.toMiddle/(car->_trkPos.seg->width));
        std::cout<<"Distance: "<<absDistToMiddle<<std::endl;
        std::cout<<"Driver distracted: "<<driverModel->getState().isDistracted<<std::endl;
        std::cout<<"Current time: "<<s->currentTime<<std::endl;
        std::cout<<"Driver state duration: "<<driverModel->getState().timeEpisodeEnd<<std::endl;
        std::cout<<"Optimal: "<<lastOptimalAction<<std::endl;
        std::cout<<"Agent action: "<<actions[lastActIdx]<<std::endl;
        std::cout<<"Driver action: "<<lastDriverAction<<std::endl;

        // planner->moveTo(lastActIdx, obs);

        // Calculate and sum up reward+
        discount *= simulator->getDiscount();
        reward  += discount * RewardCalculator::reward(*s, actions[lastActIdx]);
    }
    // int agentActionIdx = planner->getAction();
    // float agentAction = actions[agentActionIdx];

    // // Determine driver's action (discretized)
    driverModel->update(torcsState);
    float driverAction = utils::Discretizer::discretize(actions, driverModel->getAction());

    // Combine steering actions
    // car->_steerCmd = utils::Discretizer::discretize(actions, driverAction + agentAction);
    // car->_steerCmd = agentAction;
    car->_steerCmd = driverAction;

    actionsCount++;
    // lastActIdx = agentActionIdx;
    lastOptimalAction = torcsState.angle / torcsState.steerLock;
    lastDriverAction = driverAction;
}

static void
restart(tCarElt* car)
{
    runs++;
    double avgReward = reward / runs;
    std::cout<<"Restarting after "<<actionsCount<<" actions."<<std::endl;
    std::cout<<"Average reward after "<<runs<<" runs: "<<avgReward<<std::endl;
    
    car->RESET = 1;
    car->RESTART = 1;
    planner->reset();
    actionsCount = 0;
    numCallsTargetSpeed = 0;
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

