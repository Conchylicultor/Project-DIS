
#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <utility>
#include <valarray>

#include "supervisor.hpp"
#include "../common/Uniform.hpp"


/*
 * In order to simplify math operation, we use valarray to represent PSOParams
 * and convert valarray into PSOParams when needed.
 */

using Position  = std::valarray<double>;
using Speed     = std::valarray<double>;
using Modulator = Speed; // See PHI_* below
using Fitness   = double;

using PositionWithFitness = std::pair<Position, Fitness>;


/*
 * Create some random PSO parameters
 */
Position createRandomPosition()
{
    return {
        uniform(0.0, 1.0), // cohesion threshold
        uniform(0.0, 1.0), // cohesion weight
        uniform(0.0, 1.0), // separation threshold
        uniform(0.0, 1.0), // separation weight
        uniform(0.0, 1.0), // alignment weight
        uniform(0.0, 1.0), // migration weight
    };
}


/*
 * Create a random speed for a particle
 */
Speed createRandomSpeed()
{
    return {
        uniform(-1.0, 1.0), // cohesion threshold
        uniform(-1.0, 1.0), // cohesion weight
        uniform(-1.0, 1.0), // separation threshold
        uniform(-1.0, 1.0), // separation weight
        uniform(-1.0, 1.0), // alignment weight
        uniform(-1.0, 1.0), // migration weight
    };
}


/*
 * Create random PHI_* modulator for each dimension
 */
Modulator createRandomModulators()
{
    return {
        uniform(0.0, 1.0), // cohesion threshold
        uniform(0.0, 1.0), // cohesion weight
        uniform(0.0, 1.0), // separation threshold
        uniform(0.0, 1.0), // separation weight
        uniform(0.0, 1.0), // alignment weight
        uniform(0.0, 1.0), // migration weight
    };
}


/*
 * Convert a particle into PSOParams format
 */
PSOParams toParams(Position const& p)
{
    return { p[0], p[1], p[2], p[3], p[4], p[5] };
}


/*
 * Select the best particle
 */
PositionWithFitness selectBest(PositionWithFitness const& p, PositionWithFitness const& q)
{
    return q.second > p.second ? q : p;
}


/*
 * Main function.
 */
int main(int, char**)
{
    std::cout << "Loading supervisor..." << std::endl;

    reset();

    // Read the initial configuration for all robots in order to restore
    // it when measuring the fitness of some PSO settings
    RobotConfigs const initialConfigs = readAllRobotsConfig();

    // Shorthand to compute fitness
    auto computeFitness = [&initialConfigs](Position const& p) {
        // compute performance using Webots
        return simulate(initialConfigs, toParams(p));
    };

    // PSO algorithm:
    std::size_t constexpr SWARM_SIZE = 30;
    std::size_t constexpr MAX_ITERATIONS = 100;
    double constexpr OMEGA = 2;   // impact of the particle's speed (inertia)
    double constexpr PHI_P = 1.5; // impact of the personal best
    double constexpr PHI_G = 1.5; // impact of the global best
    // NOTE: PHI_P and PHI_G are scaled with x ~ U(0, 1).

    std::array<Position, SWARM_SIZE> positions; // particles' position
    std::array<Speed,    SWARM_SIZE> speeds;    // particles' speed

    std::array<PositionWithFitness, SWARM_SIZE> personalBests; // "personal" best for each particle

    // Assuming infinite range of neighbourhood
    PositionWithFitness globalBest; // "global" best

    // Generate random particles (position and speed)
    std::generate(std::begin(positions), std::end(positions), createRandomPosition);
    std::generate(std::begin(speeds),    std::end(speeds),    createRandomSpeed);

    // Update initial beliefs
    globalBest.second = std::numeric_limits<Fitness>::lowest(); // cannot be worst than that
    for (std::size_t i = 0; i < positions.size(); ++i)
    {
        auto fitness = computeFitness(positions[i]);
        std::cout << i << "-th particle: initial fitness = " << fitness << std::endl;

        // initialise personal best to the only known particle
        personalBests[i] = { positions[i], fitness };

        globalBest = selectBest(globalBest, personalBests[i]);
    }

    // Run PSO optimisation for a fixed number of iterations
    for (std::size_t t = 0; t < MAX_ITERATIONS; ++t)
    {
        for (std::size_t i = 0; i < positions.size(); ++i)
        {
            // Update i-th particle's speed and position
            auto const rP = createRandomModulators();
            auto const rG = createRandomModulators();
            speeds[i] = OMEGA * speeds[i]
                      + PHI_P * rP * (personalBests[i].first - positions[i])
                      + PHI_G * rG * (globalBest.first - positions[i]);
            positions[i] += speeds[i]; // assuming dt = 1 unit of time

            // Evaluate performance
            auto fitness = computeFitness(positions[i]);

            // Update our knowledge of the search space
            auto candidate = PositionWithFitness{ positions[i], fitness };
            personalBests[i] = selectBest(personalBests[i], candidate);
            globalBest = selectBest(globalBest, candidate);

            std::cout << "Iteration " << t
                      << ", particle " << i
                      << " has fitness " << fitness
                      << std::endl;
        }

        std::cout << "\n\n" << std::string(80, '*') << "\n"
                  << "\tEnd of iteration " << t
                  << " with best fitness of " << globalBest.second
                  << " and best settings: " << toParams(globalBest.first) << "\n"
                  << std::string(80, '*') << "\n\n"
                  << std::endl;
    }

    std::cout << "Best fitness: " << globalBest.second << "\n"
              << "Best settings: " << toParams(globalBest.first) << std::endl;

    return 0;
}

// vim: set spelllang=en_gb
// vim: set ts=4 sw=4
