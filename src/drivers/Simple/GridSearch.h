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

		tuple<bool, string, double, vector<float>, int, double, unsigned> getNextScenarios();
		tuple<bool, string, double, vector<float>, int, double, unsigned> getNextDiscountScenario();

		static GridSearch& getInstance();

		string agentScenario = INITIAL_AGENT_SCENARIO; // "planner" -> "driver" -> "optimal"
		int actionsScenarioIdx = -1; // TODO Smarter initial Call
		int discountScenarioIdx = -1; // TODO Smarter initial Call
		unsigned binsScenarioIdx = 0; 
		unsigned planningTimeScenarioIdx = 0;
		unsigned numSimsScenarioIdx = 0;

		void reset();
	private:
		GridSearch() = default;
};

inline
GridSearch& GridSearch::getInstance() {
	static GridSearch instance;
    return instance;
}

inline 
tuple<bool, string, double, vector<float>, int, double, unsigned> GridSearch::getNextScenarios() 
{	
	bool isFinished = false;
	if (actionsScenarioIdx == ((int) ACTION_SCENARIOS.size() - 1)) {
		actionsScenarioIdx = 0;
		if (agentScenario == "planner") {
			if (binsScenarioIdx == BIN_SCNEARIOS.size() - 1) {
				binsScenarioIdx = 0;
				if (planningTimeScenarioIdx == PLANNING_TIME_SCENARIOS.size() - 1) {
					planningTimeScenarioIdx = 0;
					if (numSimsScenarioIdx == NUM_SIMS_SCENARIOS.size() - 1) {
						agentScenario = "driver";
					} else {
						numSimsScenarioIdx++;
					}
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
					  PLANNING_TIME_SCENARIOS[planningTimeScenarioIdx],
					  NUM_SIMS_SCENARIOS[numSimsScenarioIdx]);
}

inline 
tuple<bool, string, double, vector<float>, int, double, unsigned> GridSearch::getNextDiscountScenario() 
{
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
					  ACTION_SCENARIOS[0], 
					  BIN_SCNEARIOS[0], 
					  PLANNING_TIME_SCENARIOS[0],
					  NUM_SIMS_SCENARIOS[0]);
}

inline 
void GridSearch::reset() 
{
	agentScenario = "planner"; // "planner" -> "driver" -> "optimal"
	binsScenarioIdx = 0;
	actionsScenarioIdx = 0;
	planningTimeScenarioIdx = 0;
	discountScenarioIdx = 0;
}

}

#endif