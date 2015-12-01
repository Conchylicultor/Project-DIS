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
    double migrationWeigth;

    // Common to all parameters: the duration of the simulation during which the fitness is evaluated
    static std::size_t const ITERATIONS = 1000; // * TIME_STEP = total duration [ms]
    // TODO increase this number
};

static PSOParams const& DEFAULT_PSOPARAMS{0.1, 0.5, 0.1, 1.0, 0.01, 0.02};

inline std::ostream& operator<<(std::ostream& out, PSOParams const& params)
{
    return out << "[\n"
               << "\tcohesion threshold   = " << params.cohesionThreshold   << "\n"
               << "\tcohesion weight      = " << params.cohesionWeight      << "\n"
               << "\tseparation threshold = " << params.separationThreshold << "\n"
               << "\tseparation weight    = " << params.spearationWeight    << "\n"
               << "\talignment weight     = " << params.alignmentWeight     << "\n"
               << "\tmigration weight     = " << params.migrationWeigth     << "\n"
               << "]";
}

#endif

// vim: set spelllang=en_gb
// vim: set ts=4 sw=4
