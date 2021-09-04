#include <iostream>
#include <vector>
#include <math.h>

#include "SimpleIni.h"

using std::vector;
using std::string;

#define CONFIG_FILE "/home/jokke/Repositories/TorcsPOMDP/experiment/Launcher/src/config.ini"
#define THRESHOLD 0.1
// #define RUNS_SPLIT_THRESHOLD 1500
// #define RUNS_SPLIT 5

static const vector<unsigned> NUM_SIMS_SCENARIOS{ 10, 100, 200, 300, 400, 500, 750, 1000, 1500, 2500, 5000, 7500, 10000 };
static const vector<double> EXP_CONST_SCENARIOS{ 0.5, 0.75, 1.5, 5, 10, 25 };
static const vector<double> DISCOUNT_SCENARIOS{ pow(THRESHOLD, (1.0/4)),   //  5
												pow(THRESHOLD, (1.0/9)),   // 10
        										pow(THRESHOLD, (1.0/24))}; // 25

bool getScenario(size_t& numSimsScenarioIdx, size_t& expConstScenarioIdx, size_t& discountScenarioIdx) 
{
	if (numSimsScenarioIdx == NUM_SIMS_SCENARIOS.size() - 1) {
		numSimsScenarioIdx = 0;
		if (expConstScenarioIdx == EXP_CONST_SCENARIOS.size() - 1) {
			expConstScenarioIdx = 0;
			if (discountScenarioIdx == DISCOUNT_SCENARIOS.size() - 1) {
				// Experiment is finished
				return false;
			} else {
				discountScenarioIdx++;
			}
		} else {
			expConstScenarioIdx++;
		}
	} else {
		numSimsScenarioIdx++;
	}

	return true;
}

void checkRC(int rc) {
	if (rc < 0) { 
		std::cout << rc << std::endl;
		throw "Configuration error"; 
	};
}

void printConfig(unsigned numSims, double expConst, double discount) {
	std::cout << std::to_string(numSims) + " Simulations, " + std::to_string(expConst) + " exploration constant, " + std::to_string(discount) + " discount" <<std::endl;
}

void writeConfig(unsigned numSims, double expConst, double discount) {
	CSimpleIniA ini;
	ini.SetUnicode();
	SI_Error rc = ini.LoadFile(CONFIG_FILE);
	checkRC(rc);

	rc = ini.SetValue("grid search", "NUM_SIMS", std::to_string(numSims).c_str());
	checkRC(rc);

	rc = ini.SetValue("grid search", "EXP_CONST", std::to_string(expConst).c_str());
	checkRC(rc);

	rc = ini.SetValue("grid search", "DISCOUNT", std::to_string(discount).c_str());
	checkRC(rc);

	rc = ini.SaveFile(CONFIG_FILE);
	checkRC(rc);
}

void launch() {
	system("screen -d -m torcs -L /home/jokke/Repositories/TorcsPOMDP/build/lib/torcs -D /home/jokke/Repositories/TorcsPOMDP/build/share/games/torcs -r /home/jokke/Repositories/TorcsPOMDP/build/share/games/torcs/config/raceman/quickrace.xml -d &");
}

int main(int argc, char *argv[])
{
	
	size_t numSimsScenarioIdx = -1; // Initially negative 
	size_t expConstScenarioIdx = 0;
	size_t discountScenarioIdx = 0;

	while(getScenario(numSimsScenarioIdx, expConstScenarioIdx, discountScenarioIdx)) {
		unsigned numSims = NUM_SIMS_SCENARIOS[numSimsScenarioIdx];
		double expConst = EXP_CONST_SCENARIOS[expConstScenarioIdx];
		double discount = DISCOUNT_SCENARIOS[discountScenarioIdx];
		printConfig(numSims, expConst, discount);
		writeConfig(numSims, expConst, discount);
		launch();
	}

	return 0;
}


