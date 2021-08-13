#ifndef _GRID_SEARCH_H_
#define _GRID_SEARCH_H_

#include <vector>
#include <tuple>
#include <cmath>

#include "Constants.h"

using std::vector;
using std::tuple;
using std::make_tuple;

namespace pomdp {

class GridSearch {
	public:
		GridSearch() = default;
		~GridSearch() = default;
		
		tuple<bool, double, vector<float>, int, double> getNextScenarios();
		tuple<bool, double, vector<float>, int, double> getNextDiscountScenario();

		void reset();

		std::string agentScenario = "planner"; // "planner" -> "driver" -> "optimal"
		int actionsScenarioIdx = -1; // TODO Smarter initial Call
		int binsScenarioIdx = 0; 
		int planningTimeScenarioIdx = 0;
		int discountScenarioIdx = -1;
};

inline tuple<bool, double, vector<float>, int, double> GridSearch::getNextScenarios() 
{
	bool isFinished = false;
	if (actionsScenarioIdx == ACTION_SCENARIOS.size() - 1) {
		actionsScenarioIdx = 0;
		if (agentScenario == "planner") {
			if (binsScenarioIdx == BIN_SCNEARIOS.size() - 1) {
				binsScenarioIdx = 0;
				if (planningTimeScenarioIdx == PLANNING_TIME_SCENARIOS.size() - 1) {
					agentScenario = "driver";
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
					  DISCOUNT_SCENARIOS[0],
					  ACTION_SCENARIOS[actionsScenarioIdx], 
					  BIN_SCNEARIOS[binsScenarioIdx], 
					  PLANNING_TIME_SCENARIOS[planningTimeScenarioIdx]);
}

inline tuple<bool, double, vector<float>, int, double> GridSearch::getNextDiscountScenario() 
{
	bool isFinished = false;
	if (discountScenarioIdx < DISCOUNT_SCENARIOS.size() - 1) {
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
					  DISCOUNT_SCENARIOS[discountScenarioIdx],
					  ACTION_SCENARIOS[0], 
					  BIN_SCNEARIOS[2], 
					  PLANNING_TIME_SCENARIOS[2]);
}

inline void GridSearch::reset() 
{
	agentScenario = "planner"; // "planner" -> "driver" -> "optimal"
	binsScenarioIdx = 0;
	actionsScenarioIdx = 0;
	planningTimeScenarioIdx = 0;
	discountScenarioIdx = 0;
}

}

#endif