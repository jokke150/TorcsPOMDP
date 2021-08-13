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

#include "Driver.h"

using namespace pomdp;

static Driver* driver;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s, tRmInfo *ReInfo); 
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

    modInfo->name    = strdup("Simple");	/* name of the module (short) */
    modInfo->desc    = strdup("");	        /* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		    /* init function */
    modInfo->gfId    = ROB_IDENT;		    /* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    driver          = new Driver();
    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
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
    driver->initTrack(track, carHandle, carParmHandle, s);
    *carParmHandle = NULL; 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
    driver->newRace(car, s);
} 

/* Drive during race. */
static void
drive(int index, tCarElt* car, tSituation *s, tRmInfo *ReInfo)
{
    driver->drive(s, ReInfo);
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
    driver->endRace(s);
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    delete driver;
}

