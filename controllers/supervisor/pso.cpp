
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

using Particle = std::valarray<double>; // i.e. the "position"
using Speed    = std::valarray<double>; // for one particle
using Fitness  = double;

using ParticleWithFitness = std::pair<Particle, Fitness>;

/*
 * Create some random PSO parameters
 */
Particle createRandomParticle()
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
 * Convert a particle into PSOParams format
 */
PSOParams particleToParams(Particle const& p)
{
    return { p[0], p[1], p[2], p[3], p[4], p[5] };
}

/*
 * Select the best particle
 */
ParticleWithFitness selectBest(ParticleWithFitness const& p, ParticleWithFitness const& q)
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
    auto computeFitness = [&initialConfigs](Particle const& p) {
        // compute performance using Webots
        return simulate(initialConfigs, particleToParams(p));
    };

    // PSO algorithm:
    std::size_t constexpr SWARM_SIZE = 30;
    std::array<Particle, SWARM_SIZE> particles; // particles' position
    std::array<Speed,    SWARM_SIZE> speeds;    // particles' speed

    std::array<ParticleWithFitness, SWARM_SIZE> personalBests; // "personal" best for each particle

    // Assuming infinite range of neighbourhood
    ParticleWithFitness globalBest; // "global" best

    // Generate random particles (position and speed)
    std::generate(std::begin(particles), std::end(particles), createRandomParticle);
    std::generate(std::begin(speeds),    std::end(speeds),    createRandomSpeed);

    // Update initial beliefs
    globalBest.second = std::numeric_limits<Fitness>::lowest(); // cannot be worst than that
    for (std::size_t i = 0; i < particles.size(); ++i) {
        auto fitness = computeFitness(particles[i]);
        std::cout << i << "-th particle: initial fitness = " << fitness << std::endl;

        // initialise personal best to the only known particle
        personalBests[i] = { particles[i], fitness };

        globalBest = selectBest(globalBest, personalBests[i]);
    }

    std::cout << "Best settings: " << particleToParams(globalBest.first) << std::endl;

    return 0;
}

