#include <vector>

namespace utils
{

class Discretizer {
	public:
        /** 
         * Discretizes the input search value according to an ordered list of bins.
         * @return bin closest to value
         */
		static float discretize(const std::vector<float>& sortedBins, float value);
};

inline
float Discretizer::discretize(const std::vector<float>& sortedBins, float value) {
    auto iterGEQ = std::lower_bound(
        sortedBins.begin(), 
        sortedBins.end(), 
        value
    );

    if (iterGEQ == sortedBins.begin()) {
        return sortedBins[0];
    }

    double a = *(iterGEQ - 1);
    double b = *(iterGEQ);

    if (fabs(value - a) < fabs(value - b)) {
        return sortedBins[iterGEQ - sortedBins.begin() - 1];
    }

    return sortedBins[iterGEQ - sortedBins.begin()];
}

}