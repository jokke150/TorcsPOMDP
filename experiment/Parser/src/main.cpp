#include <iostream>
#include <tuple>
#include <csv.hpp>
#include <statistic.h>

using namespace csv;
using std::vector;

static const int NUM_ACTIONS = 300;
static const std::string FILENAME = "b1 a1 p0";

int main(int argc, char *argv[])
{
	CSVFormat format;
	format.delimiter(',')
		  .quote(false)
      	  .header_row(0);
	CSVReader reader(FILENAME + ".csv", format);

	STATISTIC statistic;
	vector<STATISTIC> statistics(NUM_ACTIONS, statistic);

	int count;
	bool cheat;
	double reward;
	for (CSVRow& row: reader) { // Input iterator
		count = row["Count"].get<int>();
		cheat = row["Cheat"].get<bool>(); // TODO: Check for cheating
		cheat = row["Terminal"].get<bool>(); // TODO: Check for terminal state
		reward = row["Reward"].get<double>();
		statistics[count - 1].Add(reward);
	}

	std::ofstream ofs;
	auto writer = make_csv_writer_buffered(ofs);
	ofs.open (FILENAME + " mean.csv", std::ofstream::out | std::ofstream::app);
	writer << std::vector<std::string>({"Count", "Average Reward", "Standard Deviation", "Standard Error", "Min", "Max"});
	for (int i = 0; i < statistics.size(); i++) {
		auto statistic = statistics[i];
		writer << std::make_tuple(i + 1, statistic.GetMean(), statistic.GetStdDev(), statistic.GetStdErr(), statistic.GetMin(), statistic.GetMax());
	}
	writer.flush();

	return 0;
}