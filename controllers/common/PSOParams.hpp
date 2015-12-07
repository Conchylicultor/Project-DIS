#ifndef PSOPARAMS_HPP
#define PSOPARAMS_HPP

#include <cstddef>
#include <iostream>
#include <sstream>

static const int NB_SENSORS = 8; // Number of distance sensors

/*
 * Parameters to optimise
 */
struct PSOParams
{
    double avoidanceWeights[NB_SENSORS];
    double cohesionWeight;
    double alignmentWeight;
    double migrationWeight;

    // Common to all parameters: the duration of the simulation during which the fitness is evaluated
    static std::size_t const SIMULATION_STEPS = 1000; // * TIME_STEP = total duration [ms]
};


inline std::ostream& operator<<(std::ostream& out, PSOParams const& params)
{
    out << "[\n"
        << "\tavoidance weights    = [ ";
    for (auto w : params.avoidanceWeights)
        out << w << " ";
    out << "]\n"
        << "\tcohesion weight      = " << params.cohesionWeight << "\n"
        << "\talignment weight     = " << params.alignmentWeight << "\n"
        << "\tmigration weight     = " << params.migrationWeight << "\n"
        << "]";
    return out;
}


inline std::istream& operator>>(std::istream& in, PSOParams& params)
{
    // Ignore first line (`[`)
    std::string line;
    std::getline(in, line);

    // Avoidance:
    std::getline(in, line);
    auto idx1 = line.find('[') + 1;
    auto idx2 = line.find(']');
    line = line.substr(idx1, idx2 - idx1 - 1);
    std::istringstream ss(line);
    for (auto& w : params.avoidanceWeights)
        ss >> w;

    // Cohesion:
    std::getline(in, line);
    idx1 = line.find('=');
    line = line.substr(idx1 + 1);
    params.cohesionWeight = std::stod(line);

    // Alignment:
    std::getline(in, line);
    idx1 = line.find('=');
    line = line.substr(idx1 + 1);
    params.alignmentWeight = std::stod(line);

    // Migration:
    std::getline(in, line);
    idx1 = line.find('=');
    line = line.substr(idx1 + 1);
    params.migrationWeight = std::stod(line);

    // Ignore last line (`]`)
    std::getline(in, line);

    return in;
}

static const unsigned int NB_PARAMS = 3 + NB_SENSORS;

// Normalize using x = (x-min) / (max-min)
static double minMaxParams[NB_PARAMS][2] = { { 0.0, 1.0 },   // avoidanceWeights[0]
                                             { 0.0, 1.0 },   // avoidanceWeights[1]
                                             { 0.0, 1.0 },   // avoidanceWeights[2]
                                             { 0.0, 1.0 },   // avoidanceWeights[3]
                                             { 0.0, 1.0 },   // avoidanceWeights[4]
                                             { 0.0, 1.0 },   // avoidanceWeights[5]
                                             { 0.0, 1.0 },   // avoidanceWeights[6]
                                             { 0.0, 1.0 },   // avoidanceWeights[7]
                                             { 0.0, 1.0 },   // cohesionWeight
                                             { 0.0, 1.0 },   // alignmentWeight
                                             { 0.0, 1.0 } }; // migrationWeigth

/*
 * Normalize the robots params to [0-1] before applying PSO
 */
inline PSOParams normalizeParams(const PSOParams &paramsToNormalize)
{
  PSOParams params = paramsToNormalize;
  double *paramsArray = reinterpret_cast<double*>(&params);
  for(unsigned int i = 0 ; i < NB_PARAMS ; ++i)
  {
    // X = (X-min)/(max-min)
    paramsArray[i] = (paramsArray[i] - minMaxParams[i][0])/(minMaxParams[i][1] - minMaxParams[i][0]);
  }
  return params;
}


/*
 * "De-normalize" the params before sending them to the epucks
 */
inline PSOParams restoreParams(const PSOParams &paramsToRestore)
{
  PSOParams params = paramsToRestore;
  double *paramsArray = reinterpret_cast<double*>(&params);
  for(unsigned int i = 0 ; i < NB_PARAMS ; ++i)
  {
    // X = (X*(max-min)) + min
    paramsArray[i] = (paramsArray[i] * (minMaxParams[i][1] - minMaxParams[i][0])) + minMaxParams[i][0];
  }
  return params;
}

#endif

// vim: set spelllang=en_gb
// vim: set ts=4 sw=4
