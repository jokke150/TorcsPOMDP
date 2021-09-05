#include <iostream>
#include <vector>
#include <math.h>
#include <sstream>

#include <chrono>
#include <thread>

#include "SimpleIni.h"

using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

using std::vector;
using std::string;

#define CONFIG_FILE "/home/jokke/Repositories/TorcsPOMDP/experiment/Launcher/src/config.ini"
#define THRESHOLD 0.1

static vector<unsigned> NUM_SIMS_SCENARIOS;
static vector<double> EXP_CONST_SCENARIOS;
static vector<double> DISCOUNT_SCENARIOS;

template <typename T>
vector<T> parseScenarios(string scenariosString) {
	vector<T> scenarios;
	std::istringstream iss(scenariosString);
    string item;
    while (std::getline(iss, item, ',')) {
		std::stringstream ss(item);
		T scenario;
		ss >> scenario;
        scenarios.push_back(scenario);
    }
	return scenarios;
}

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

void printConfig(string scenarioName, unsigned numSims, double expConst, double discount) {
	std::cout << scenarioName + ": " + std::to_string(numSims) + " Simulations, " + std::to_string(expConst) + " exploration constant, " + std::to_string(discount) + " discount" <<std::endl;
}

void readConfig(CSimpleIniA& ini) {
	ini.SetUnicode();
	SI_Error rc = ini.LoadFile(CONFIG_FILE);
	checkRC(rc);
}

void writeConfig(CSimpleIniA& ini, unsigned numSims, double expConst, double discount) {
	SI_Error rc;
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
	sleep_for(seconds(10));
}

int main(int argc, char *argv[])
{
	CSimpleIniA ini;
	readConfig(ini);

	string scenarioName = ini.GetValue("manual", "SCENARIO_NAME");
	NUM_SIMS_SCENARIOS = parseScenarios<unsigned>(ini.GetValue("manual", "NUM_SIMS_SCENARIOS"));
	EXP_CONST_SCENARIOS = parseScenarios<double>(ini.GetValue("manual", "EXP_CONST_SCENARIOS"));
	DISCOUNT_SCENARIOS = parseScenarios<double>(ini.GetValue("manual", "DISCOUNT_SCENARIOS"));
	
	size_t numSimsScenarioIdx = -1; // Initially negative 
	size_t expConstScenarioIdx = 0;
	size_t discountScenarioIdx = 0;

	while(getScenario(numSimsScenarioIdx, expConstScenarioIdx, discountScenarioIdx)) {
		unsigned numSims = NUM_SIMS_SCENARIOS[numSimsScenarioIdx];
		double expConst = EXP_CONST_SCENARIOS[expConstScenarioIdx];
		double discount = DISCOUNT_SCENARIOS[discountScenarioIdx];
		printConfig(scenarioName, numSims, expConst, discount);
		writeConfig(ini, numSims, expConst, discount);
		launch();
	}

	return 0;
}


