#include <iostream>
#include <fstream>
#include <filesystem>
#include <regex>
#include <tuple>
#include <map>
#include <csv.hpp>
#include <statistic.h>
using namespace csv;
namespace fs = std::filesystem;
using std::vector;
using std::string;

static const int NUM_RUNS = 10;
static const int NUM_ACTIONS = 1000;
static const string PATH = "/home/jokke/pCloudDrive/Utrecht/Thesis/Data/";
static const string FOLDER = "Over Correct + Noise - 10 runs - 1000 actions - discount horizon 5";
// "Simulations - 10 runs - 100 actions - discount horizon 5 - exp const 0.75";
// "Over Correct - 10 runs - 1000 actions - discount horizon 5";
// "Over Correct + Noise - 10 runs - 1000 actions - discount horizon 5";
static const string EXT = ".csv";

int main(int argc, char *argv[])
{
	std::map<int, string> scenarioBySimulations;
    for (auto &p : fs::directory_iterator(PATH + FOLDER + "/"))
    {
        if (p.path().extension() == EXT) {
			string scenario = p.path().stem().string();
			std::regex rgx("\\ss(\\d+)\\s");
			std::smatch match;
			std::regex_search(scenario, match, rgx);
			int simulations = std::stoi(match[1]);
			scenarioBySimulations[simulations] = scenario;
		}
    }

	CSVFormat format;
	format.delimiter(',')
		  .quote(false)
      	  .header_row(0);

	std::ofstream ofs;
	auto writer = make_csv_writer(ofs);
	ofs.open (FOLDER + " mean.csv", std::ofstream::out | std::ofstream::app);
	writer << std::vector<string>({"Simulations", "Average Reward", "Standard Deviation", "Standard Error", "Min", "Max", "Cheat", "Terminal"});
	
	std::map<int, string> lineBySimulations;

	for (const auto & [simulations, scenario]  : scenarioBySimulations) {
		std::cout << scenario << "\n";

		CSVReader reader(PATH + FOLDER + "/" + scenario + ".csv", format);

		STATISTIC statistic;

		std::map<int, int> firstCheatByRun;
		std::vector<int> terminalRuns;
		int lastRun = 1;
		double lastReward; 
		for (CSVRow& row: reader) { // Input iterator
			int run = row["Run"].get<int>();
			if (lastRun != run) {
				statistic.Add(lastReward);
				lastRun = run;
			}

			int count = row["Count"].get<int>();
			bool cheat = row["Cheat"].get<string>() == "cheat";
			if (cheat && !firstCheatByRun.count(run)) {
				firstCheatByRun[run] = count + 1;
			}

			bool terminal = row["Terminal"].get<bool>();
			if (terminal) {
				terminalRuns.push_back(run);
			}

			lastReward = row["Reward"].get<double>();
		}

		statistic.Add(lastReward);

		writer << std::make_tuple(simulations, 
								  statistic.GetMean(), 
								  statistic.GetStdDev(), 
								  statistic.GetStdErr(), 
								  statistic.GetMin(), 
								  statistic.GetMax(), 
								  firstCheatByRun.size(), 
								  terminalRuns.size());
	}
	return 0;
	

	// STATISTIC statistic;
	// vector<STATISTIC> statistics(NUM_ACTIONS, statistic);

	// std::map<int, int> firstCheatByRun;
	// std::vector<int> terminalRuns;
	// for (CSVRow& row: reader) { // Input iterator
	// 	int run = row["Run"].get<int>();
	// 	int count = row["Count"].get<bool>();

	// 	double reward = row["Reward"].get<double>();
	// 	statistics[count - 1].Add(reward);

	// 	bool cheat = row["Cheat"].get<bool>();
	// 	if (cheat && !firstCheatByRun[run]) {
	// 		firstCheatByRun[run] = count + 1;
	// 	}

	// 	bool terminal = row["Terminal"].get<bool>();
	// 	terminalRuns.push_back(run);
	// }

	// std::ofstream ofs;
	// auto writer = make_csv_writer_buffered(ofs);
	// ofs.open (FILENAME + " mean.csv", std::ofstream::out | std::ofstream::app);
	// writer << std::vector<string>({"Count", "Average Reward", "Standard Deviation", "Standard Error", "Min", "Max", "Cheat", "Terminal"});
	// for (int i = 0; i < statistics.size(); i++) {
	// 	auto statistic = statistics[i];
	// 	writer << std::make_tuple(i + 1, statistic.GetMean(), statistic.GetStdDev(), statistic.GetStdErr(), statistic.GetMin(), statistic.GetMax(), firstCheatByRun.size(), terminalRuns.size());
	// }
	// writer.flush();

	// return 0;
}