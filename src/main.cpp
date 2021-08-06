#include <iostream>

#include <tgfclient.h>
#include <raceman.h>

static tModList *reEventModList = 0;
static tModList *ReRaceModList = 0;
static tRmInfo	*ReInfo;


void initModules()
{
	GfInitClient(); // TODO

	ReInfo = (tRmInfo *)calloc(1, sizeof(tRmInfo));
	ReInfo->s = (tSituation *)calloc(1, sizeof(tSituation));
	ReInfo->modList = &ReRaceModList;

	const int BUFSIZE = 1024;
	char buf[BUFSIZE];

	snprintf(buf, BUFSIZE, "%s%s", GetLocalDir(), "config/raceengine.xml");

	ReInfo->_reParam = GfParmReadFile(buf, GFPARM_RMODE_REREAD | GFPARM_RMODE_CREAT);

	GfOut("Loading Track Loader...\n");
	const char* trk_dllname = GfParmGetStr(ReInfo->_reParam, "Modules", "track", "");
	snprintf(buf, BUFSIZE, "%smodules/track/%s.%s", GetLibDir (), trk_dllname, DLLEXT);
	if (GfModLoad(0, buf, &reEventModList)) throw std::runtime_error("Could not load track loader.");
	reEventModList->modInfo->fctInit(reEventModList->modInfo->index, &ReInfo->_reTrackItf);

	const char* simu_dllname = GfParmGetStr(ReInfo->_reParam, "Modules", "simu", "");
	snprintf(buf, BUFSIZE, "%smodules/simu/%s.%s", GetLibDir (), simu_dllname, DLLEXT);
	if (GfModLoad(0, buf, &ReRaceModList)) throw std::runtime_error("Could not load simu.");
	ReRaceModList->modInfo->fctInit(ReRaceModList->modInfo->index, &ReInfo->_reSimItf);
}

void initTrack()
{
	const int BUFSIZE = 1024;
	char buf[BUFSIZE];
	const char* trackName = "brondehach";
	const char* catName = "road";

	snprintf(buf, BUFSIZE, "Loading Track %s...", trackName);
	snprintf(buf, BUFSIZE, "torcs/build/share/games/torcs/tracks/%s/%s/%s.%s", catName, trackName, trackName, TRKEXT);
	ReInfo->track = ReInfo->_reTrackItf.trkBuild(buf);
}

void initCars()
{
	int nCars = 0;

	FREEZ(ReInfo->carList);
	ReInfo->carList = (tCarElt*)calloc(nCars, sizeof(tCarElt));
	FREEZ(ReInfo->rules);
	ReInfo->rules = (tRmCarRules*)calloc(nCars, sizeof(tRmCarRules));

	void *params = ReInfo->params;
	const int BUFSIZE = 1024;
	char buf[BUFSIZE], path[BUFSIZE];

	const char* cardllname = "Simple";
	int robotIdx = 0;
	snprintf(path, BUFSIZE, "%sdrivers/%s/%s.%s", GetLibDir (), cardllname, cardllname, DLLEXT);

	if (GfModLoad(0, buf, &reEventModList)) throw std::runtime_error("Could not load driver.");

	if (GfModLoad(CAR_IDENT, path, ReInfo->modList)) {
		GfTrace("Pb with loading %s driver\n", path);
		return;
	}



	ReInfo->_reSimItf.init(nCars, ReInfo->track, ReInfo->raceRules.fuelFactor, ReInfo->raceRules.damageFactor);
}

int main(int argc, char *argv[])
{
	LinuxSpecInit();
	initTrack();
	std::cout << "Hello world!" << std::endl;
}
