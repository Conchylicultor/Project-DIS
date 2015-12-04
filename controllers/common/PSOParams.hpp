#ifndef PSOPARAMS_HPP
#define PSOPARAMS_HPP

#include <cstddef>
#include <ostream>

/*
 * Parameters to optimise
 */
struct PSOParams
{
    // Cohesion

    // Threshold to activate cohesion rule. This represents the minimal distance that
    // triggers attraction toward the centre of mass of the flock.
    double cohesionThreshold;
    double cohesionWeight;

    // Separation

    // Threshold to activate dispersion rule. This represents the minimal allowed distance between
    // two boids before they try to avoid each other.
    double separationThreshold;
    double spearationWeight;

    // Alignment
    double alignmentWeight;

    // Migration
    double migrationWeight;
};

// Common to all parameters: the duration of the simulation during which the fitness is evaluated
static std::size_t const PSO_ITERATIONS = 1000; // * TIME_STEP = total duration [ms]
// TODO increase this number

inline std::ostream& operator<<(std::ostream& out, PSOParams const& params)
{
    return out << "[\n"
               << "\tcohesion threshold   = " << params.cohesionThreshold   << "\n"
               << "\tcohesion weight      = " << params.cohesionWeight      << "\n"
               << "\tseparation threshold = " << params.separationThreshold << "\n"
               << "\tseparation weight    = " << params.spearationWeight    << "\n"
               << "\talignment weight     = " << params.alignmentWeight     << "\n"
               << "\tmigration weight     = " << params.migrationWeight     << "\n"
               << "]";
}



static const unsigned int NB_PARAMS = 6;

// Normalize using x = (x-min) / (max-min)
static double minMaxParams[NB_PARAMS][2] = {{0.0, 1.0}, // cohesionThreshold
                                            {0.0, 1.0}, // cohesionWeight
                                            {0.0, 1.0}, // separationThreshold
                                            {0.0, 1.0}, // spearationWeight
                                            {0.0, 1.0}, // alignmentWeight
                                            {0.0, 1.0}}; //migrationWeigth

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
