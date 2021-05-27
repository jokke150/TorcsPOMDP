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

#include "sensors.h"

#include <simu.h>

static tTrack	*curTrack;

// SENSORS
static tdble oldAccel;
static tdble oldBrake;
static tdble oldSteer;
static tdble oldClutch;
static tdble prevDist;
static tdble distRaced;

static int oldGear;

#define RACE_RESTART 1
static int RESTARTING;

#define __NUM_SENSORS__ 5
#define __SENSORS_RANGE__ 100
static Sensors *trackSens;
static float trackSensAngle[__NUM_SENSORS__] = { -80, -40, 0, 40, 80 };
#define __NUM_SENSOR_BINS__ 15
static float sensorDistanceBins[__NUM_SENSOR_BINS__] = {0.5, 1, 1.5, 2, 2.5, 3.5, 5, 7.5, 10, 15, 20, 25, 30, 50, 100}; // Up-to and including distance, beyond -> -1
#define __NUM_MIDDLE_BINS__ 13
static float middleDistanceBins[__NUM_MIDDLE_BINS__] = {-1.0, -0.7, -0.5, -0.3, -0.15, -0.05, 0, 0.05, 0.15, 0.3, 0.5, 0.7, 1.0}; // Up-to and including distance, beyond -> -1
#define __NUM_ANGLE_BINS__ 10 // Must be an even number
static float * getAngleBins() {
        static float bins[__NUM_ANGLE_BINS__];
        int mult = -(__NUM_ANGLE_BINS__ / 2) + 1;
        for (float & bin : bins) {
                bin = mult * (PI / (__NUM_ANGLE_BINS__ / 2));
                mult++;
        }
        return bins;
    }
static float *angleBins = getAngleBins(); // Up-to and including angle, from -PI to PI

static unsigned long total_tics;

// GENERATIVE MODEL
tModList *SimpleModList = 0;
tSimItf	simItf;

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

    // Initialization of track sensors
    trackSens = new Sensors(car, __NUM_SENSORS__);
    for (int i = 0; i < __NUM_SENSORS__; ++i) {
    	trackSens->setSensor(i, trackSensAngle[i], __SENSORS_RANGE__);
	}
    prevDist = -1;
} 

/* Drive during race. */
static void
drive(int index, tCarElt* car, tSituation *s, tRmInfo *ReInfo)
{
    total_tics++;

    // computing distance to middle
    float dist_to_middle = 2*car->_trkPos.toMiddle/(car->_trkPos.seg->width);
    dist_to_middle = Discretizer::search(middleDistanceBins, 0, __NUM_MIDDLE_BINS__ - 1, dist_to_middle);

    // computing the car angle wrt the track axis
    float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI
    angle = Discretizer::search(angleBins, 0, __NUM_ANGLE_BINS__ - 1, angle);

    // update the value of track sensors only as long as the car is inside the track
    float trackSensorOut[__NUM_SENSORS__];
    if (dist_to_middle <= 1.0 && dist_to_middle >= -1.0) {
        trackSens->sensors_update();
        for (int i = 0; i < __NUM_SENSORS__; ++i) {
            trackSensorOut[i] = trackSens->getSensorOutDiscrete(i, sensorDistanceBins, __NUM_SENSOR_BINS__);
        }
    } else {
        for (int i = 0; i < __NUM_SENSORS__; ++i) {
            trackSensorOut[i] = -1;
        }
    }

    // float wheelSpinVel[4];
    // for (int i = 0; i < 4; ++i) {
    //     wheelSpinVel[i] = car->_wheelSpinVel(i);
    // }

    // if (prevDist < 0) {
	//     prevDist = car->race.distFromStartLine;
    // }
    // float curDistRaced = car->race.distFromStartLine - prevDist;
    // prevDist = car->race.distFromStartLine;
    // if (curDistRaced > 100) {
	//     curDistRaced -= curTrack->length;
    // }
    // if (curDistRaced < -100) {
	//     curDistRaced += curTrack->length;
    // }
    // distRaced += curDistRaced;

    car->_accelCmd = oldAccel;
    car->_brakeCmd = oldBrake;
    car->_gearCmd  = oldGear;
    car->_steerCmd = oldSteer;
    car->_clutchCmd = oldClutch;

    // ...

    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    /*  
     * modify the 
     * car->_steerCmd 
     * car->_accelCmd 
     * car->_brakeCmd 
     * car->_gearCmd 
     * car->_clutchCmd 
     */ 

    const float SC = 1.0;

    tSituation sitCopy = new tSituation(s);

    // TODO: Initialize own Simulation module in order to build tree independently 
    loadSimu(ReInfo, sitCopy.cars);

    tCarElt* carCopy = sitCopy.cars[0];

    carCopy->ctrl.steer = angle / carCopy->_steerLock;
    carCopy->ctrl.gear = 1; // first gear
    carCopy->ctrl.accelCmd = 0.3; // 30% accelerator pedal

    for (float i = 0; i<= RCM_MAX_DT_ROBOTS; i += RCM_MAX_DT_SIMU) {
        simItf.update(&sitCopy, RCM_MAX_DT_SIMU, -1);  // TODO: Use correct time interval to plan ahead
    }

    tSituation sitTest = sitCopy;

    car->ctrl.steer = angle / car->_steerLock;
    car->ctrl.gear = 1; // first gear
    car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
    car->ctrl.brakeCmd = 0.0; // no brakes
}

static void loadSimu(tRmInfo *ReInfo, tCarElt **cars) {
    if (!SimpleModList) {
        const int BUFSIZE = 1024;
        char buf[BUFSIZE];

        const char* dllname = "simuv2";
        snprintf(buf, BUFSIZE, "%smodules/simu/copy/%s.%s", GetLibDir (), dllname, DLLEXT);
        if (GfModLoad(0, buf, &SimpleModList)) throw std::runtime_error("Could not load simu.");
        SimpleModList->modInfo->fctInit(SimpleModList->modInfo->index, &simItf);

        simItf.init(ReInfo->s->_ncars, ReInfo->track, ReInfo->raceRules.fuelFactor, ReInfo->raceRules.damageFactor);
    }
    for (int i = 0; i < ReInfo->s->_ncars; i++) {
        simItf.config(cars[i], ReInfo);
    }
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
    RESTARTING=0;
    if (trackSens) {
        delete trackSens;
        trackSens = NULL;
    }
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    RESTARTING=0;
    if (trackSens) {
        delete trackSens;
        trackSens = NULL;
    }
}

