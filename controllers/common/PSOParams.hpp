#ifndef PSOPARAMS_HPP
#define PSOPARAMS_HPP

/*
 * Parameters to optimise
 */
struct PSOParams
{
    // Defaults settings
    PSOParams() :
        cohesionThreshold(0.1),
        cohesionWeight(0.5),
        separationThreshold(0.1),
        spearationWeight(1.0),
        alignmentWeight(0.01),
        migrationWeigth(0.02)
    {}

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
};

#endif

// vim: set spelllang=en_gb
// vim: set ts=4 sw=4
