#ifndef _CONFIG_READER_H_
#define _CONFIG_READER_H_
#include <iostream>

#include <vector>
#include <tuple>
#include <string>
#include <sstream>
#include <iterator>

#include "SimpleIni.h"

using std::vector;
using std::tuple;
using std::make_tuple;
using std::string;

namespace utils {

class ConfigReader {
	public:
		ConfigReader() = delete;
		~ConfigReader() = delete;
        ConfigReader(ConfigReader const&) = delete;
        void operator=(ConfigReader const&)  = delete;
		
		/**
		 * @return agentScenario, targetRuns, targetActions, numSims, 
		 * 		   agentActions, driverActions, discount, expConst,
		 * 		   driverInitAtt, driverOverCorrect, driverNoise, preferActions
		 */
		static tuple<string, string, unsigned, unsigned, unsigned, vector<float>, vector<float>, double, double, bool, bool, bool, bool> getConfig();	

	private:
		static vector<float> parseActions(string actionsString);
};

inline 
tuple<string, string, unsigned, unsigned, unsigned, vector<float>, vector<float>, double, double, bool, bool, bool, bool> 
ConfigReader::getConfig() 
{	
	CSimpleIniA ini;
	ini.SetUnicode();
	SI_Error rc = ini.LoadFile("/home/jokke/Repositories/TorcsPOMDP/experiment/Launcher/src/config.ini");
	if (rc < 0) { 
		std::cout << rc << std::endl;
		throw "Configuration reader error"; 
	};

	string scenarioName = ini.GetValue("manual", "SCENARIO_NAME");
	string agentScenario = ini.GetValue("manual", "AGENT_SCENARIO");
	unsigned targetRuns = ini.GetLongValue("manual", "TARGET_RUNS");
	unsigned targetActions = ini.GetLongValue("manual", "TARGET_ACTIONS");
	vector<float> agentActions = parseActions(ini.GetValue("manual", "AGENT_ACTIONS"));
	vector<float> driverActions = parseActions(ini.GetValue("manual", "DRIVER_ACTIONS"));
	bool driverInitAtt = ini.GetBoolValue("manual", "INITIAL_ATTENTIVE");
	bool driverOverCorrect = ini.GetBoolValue("manual", "DRIVER_OVER_CORRECT");
	bool driverNoise = ini.GetBoolValue("manual", "DRIVER_ACTION_NOISE");
	bool preferActions = ini.GetBoolValue("manual", "PREFERRED_ACTIONS");

	unsigned numSims = ini.GetLongValue("grid search", "NUM_SIMS");
	double expConst = ini.GetDoubleValue("grid search", "EXP_CONST");
	double discount = ini.GetDoubleValue("grid search", "DISCOUNT");

	return make_tuple(scenarioName, agentScenario, targetRuns, targetActions, 
					  numSims, agentActions, driverActions, discount, expConst,
		 			  driverInitAtt, driverOverCorrect, driverNoise, preferActions);
}

inline
vector<float> ConfigReader::parseActions(string actionsString) {
	vector<float> actions;
	std::istringstream iss(actionsString);
    string item;
    while (std::getline(iss, item, ',')) {
		float action = std::stof(item);
        actions.push_back(action);
    }
	return actions;
}

}

#endif