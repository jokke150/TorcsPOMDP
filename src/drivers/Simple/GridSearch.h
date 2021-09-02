#ifndef _GRID_SEARCH_H_
#define _GRID_SEARCH_H_

#include <vector>
#include <tuple>
#include <cmath>

#include "Constants.h"

using std::vector;
using std::tuple;
using std::make_tuple;
using std::string;

namespace pomdp {

class GridSearch {
	public:
        GridSearch(GridSearch const&) = delete;
        void operator=(GridSearch const&)  = delete;
		~GridSearch() = default;

		tuple<bool, string, double, vector<float>, int, unsigned, double> getNextScenarios();
		tuple<bool, string, double, vector<float>, int, unsigned, double> getNextDiscountScenario();

		static GridSearch& getInstance();

		string agentScenario = INITIAL_AGENT_SCENARIO; // "planner" -> "driver" -> "optimal"
		int discountScenarioIdx = -1; // TODO Smarter initial Call
		int actionsScenarioIdx = -1; // TODO Smarter initial Call
		unsigned binsScenarioIdx = 0; 
		unsigned numSimsScenarioIdx = 0;
		unsigned expConstScenarioIdx = 0;
	private:
		GridSearch() = default;
};

inline
GridSearch& GridSearch::getInstance() {
	static GridSearch instance;
    return instance;
}

inline 
tuple<bool, string, double, vector<float>, int, unsigned, double> GridSearch::getNextScenarios() 
{	
	bool isFinished = false;
	if (actionsScenarioIdx == ((int) ACTION_SCENARIOS.size() - 1)) {
		actionsScenarioIdx = 0;
		if (agentScenario == "planner") {
			if (binsScenarioIdx == BIN_SCNEARIOS.size() - 1) {
				binsScenarioIdx = 0;
				if (numSimsScenarioIdx == NUM_SIMS_SCENARIOS.size() - 1) {
					numSimsScenarioIdx = 0;
					if (expConstScenarioIdx == EXP_CONST_SCENARIOS.size() - 1) {
						agentScenario = "driver";
					} else {
						expConstScenarioIdx++;
					}
				} else {
					numSimsScenarioIdx++;
				}
			} else {
				binsScenarioIdx++;
			}
		} else if (agentScenario == "driver") {
			agentScenario = "optimal";
		} else {
			// Experiment is finished
			isFinished = true;
		}
	} else{
		actionsScenarioIdx++;
	}

	return make_tuple(isFinished,
					  agentScenario,
					  DEFAULT_DISCOUNT,
					  ACTION_SCENARIOS[actionsScenarioIdx], 
					  BIN_SCNEARIOS[binsScenarioIdx], 
					  NUM_SIMS_SCENARIOS[numSimsScenarioIdx],
					  EXP_CONST_SCENARIOS[expConstScenarioIdx]);
}

inline 
tuple<bool, string, double, vector<float>, int, unsigned, double> GridSearch::getNextDiscountScenario() 
{
	actionsScenarioIdx = 0;
	binsScenarioIdx = 0; 
	numSimsScenarioIdx = 0;
	expConstScenarioIdx = 0;

	bool isFinished = false;
	if (discountScenarioIdx < ((int) DISCOUNT_SCENARIOS.size() - 1)) {
		discountScenarioIdx++;
	} else {
		if (agentScenario == "planner") {
			discountScenarioIdx = 0;
			agentScenario = "driver";
		} else if (agentScenario == "driver") {
			discountScenarioIdx = 0;
			agentScenario = "optimal";
		} else {
			isFinished = true;
		}
	}

	return make_tuple(isFinished,
					  agentScenario,
					  DISCOUNT_SCENARIOS[discountScenarioIdx],
					  ACTION_SCENARIOS[actionsScenarioIdx], 
					  BIN_SCNEARIOS[binsScenarioIdx], 
					  NUM_SIMS_SCENARIOS[numSimsScenarioIdx],
					  EXP_CONST_SCENARIOS[expConstScenarioIdx]);
}

}

#endif