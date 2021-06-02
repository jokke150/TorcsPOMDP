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

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

#include <simu.h>

#include "TorcsSimulator.hpp"
#include "Pomcp.hpp"

using namespace pomcp;
using namespace pomdp;

static tTrack *curTrack;

static tdble oldAccel;
static tdble oldBrake;
static tdble oldSteer;
static tdble oldClutch;
static tdble prevDist;
static tdble distRaced;

static int oldGear;

#define RACE_RESTART 1
static int RESTARTING;

static std::vector<Action> discreteSteeringActions{-1.0, -0.8, -0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6, 0.8, 1.0};

static unsigned long total_tics;

// GENERATIVE MODEL
static tModList *SimpleModList = 0;
static tSimItf *simItf = nullptr;

// POMCP
static TorcsSimulator *simulator = nullptr;
static PomcpPlanner<State, Observation, Action, pomcp::VectorBelief<State>> *planner = nullptr;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s, tRmInfo *ReInfo); 
static void loadSimu(tRmInfo *ReInfo, tCarElt **cars);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 


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
    total_tics = 0;
    prevDist = -1;
} 

/* Drive during race. */
static void
drive(int index, tCarElt* car, tSituation *s, tRmInfo *ReInfo)
{
    total_tics++;

    // Constant actions (Cannot be influenced by planner)
    car->_accelCmd = 0.3; // 30% accelerator pedal 
    car->_brakeCmd = 0.0; // no brakes 
    car->_gearCmd = 1; // first gear
    car->_clutchCmd = 0.0 // no clutch

    static State state{ *s };
    tSituation sitCopy = new tSituation(*s);
    loadSimu(ReInfo, sitCopy.cars);

    if (!simulator) {
        simulator = new TorcsSimulator{ discreteSteeringActions, *simItf, sitCopy, ReInfo };
        delete planner;
        planner = new PomcpPlanner<State, Observation, Action, pomcp::VectorBelief<State>>{ *simulator, PLANNING_TIME, RESAMPLING_TIME, THRESHOLD, EXPLORATION_CTE, PARTICLES };
    }

    // Planning
    
    Action action = planner->getAction();
    planner.computeInfo(size,depth);
    std::cout<<"Size: "<<size<<std::endl;
    std::cout<<"Depth: "<<depth<<std::endl;
    planner.moveTo(a0,z1); // TODO: Only execute after first action has been performed

    // TODO: Calculate and sum up reward

    // Steering action
    car->_steerCmd
}

static void loadSimu(tRmInfo *ReInfo, tCarElt **cars) {
    if (!SimpleModList) {
        const int BUFSIZE = 1024;
        char buf[BUFSIZE];

        const char* dllname = "simuv2";
        snprintf(buf, BUFSIZE, "%smodules/simu/copy/%s.%s", GetLibDir (), dllname, DLLEXT);
        if (GfModLoad(0, buf, &SimpleModList)) throw std::runtime_error("Could not load simu.");
        SimpleModList->modInfo->fctInit(SimpleModList->modInfo->index, &simItf);

        if (!simItf) simItf = new tSimItf;
        simItf->init(ReInfo->s->_ncars, ReInfo->track, ReInfo->raceRules.fuelFactor, ReInfo->raceRules.damageFactor);
    }
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
    RESTARTING=0;
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    RESTARTING=0;
    delete planner;
    delete simulator;
    delete simItf;
}

