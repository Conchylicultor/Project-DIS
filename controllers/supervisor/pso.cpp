
#include <algorithm>
#include <array>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
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
using Modulator = Speed; // See PHI_* in main PSO loop
using Fitness   = double;

using PositionWithFitness = std::pair<Position, Fitness>;

using Evalutor  = std::function<Fitness(Position const&)>;

std::size_t constexpr SWARM_SIZE = 15;
std::size_t constexpr MAX_ITERATIONS = 10000;
double constexpr OMEGA = 0.7;  // impact of the particle's speed (inertia)
double constexpr PHI_P = 0.15; // impact of the personal best
double constexpr PHI_G = 0.25; // impact of the global best
// NOTE: PHI_P and PHI_G are scaled with x ~ U(0, 1).

std::string const& SAVE_FILE = "results.txt";

using Positions = std::array<Position, SWARM_SIZE>;
using Speeds    = std::array<Speed,    SWARM_SIZE>;
using PwFs      = std::array<PositionWithFitness, SWARM_SIZE>;

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
    static_assert(NB_PARAMS == 6, "wrong size");
}


/*
 * Create a random speed for a particle
 */
Speed createRandomSpeed()
{
    return {
        uniform(-0.1, 0.1), // cohesion threshold
        uniform(-0.1, 0.1), // cohesion weight
        uniform(-0.1, 0.1), // separation threshold
        uniform(-0.1, 0.1), // separation weight
        uniform(-0.1, 0.1), // alignment weight
        uniform(-0.1, 0.1), // migration weight
    };
    static_assert(NB_PARAMS == 6, "wrong size");
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
    static_assert(NB_PARAMS == 6, "wrong size");
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
 * Perform random initialisation of the particles
 */
void initilisePSO(Evalutor const& computeFitness, Positions& positions, Speeds& speeds,
                  PwFs& personalBests, PositionWithFitness& globalBest,
                  PositionWithFitness& absoluteBest)
{
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

    absoluteBest = globalBest;
}


template <typename T>
std::ostream& operator<<(std::ostream& out, std::valarray<T> const& v)
{
    for (auto coord : v)
        out << coord << " ";
    return out;
}


template <typename T>
std::istream& operator>>(std::istream& in, std::valarray<T>& v)
{
    for (auto& coord : v)
        in >> coord;
    return in;
}


/*
 * Check whether a file exists or not
 */
bool isThereFile(std::string const& filename)
{
    std::ifstream infile(filename);
    return infile.good();
}


/*
 * Copy the content of a file to another one
 */
void copyFile(std::string const& source, std::string const& dest)
{
    std::ifstream src(source, std::ios::binary);
    std::ofstream dst(dest, std::ios::binary);

    dst << src.rdbuf();
}


/*
 * Save current PSO states to a file; the format of the file is really simple and
 * could benefit from using a proper serialisation library.
 */
void saveState(std::string const& filename, std::size_t t, Positions const& positions,
               Speeds const& speeds, PwFs const& personalBests,
               PositionWithFitness const& globalBest, PositionWithFitness const& absoluteBest)
{
    // We don't throw exception from this function: we don't want to kill the whole program
    // for that.

    std::ofstream out(filename);
    if (!out)
    {
        std::cerr << "Couldn't open " << filename << " for writing" << std::endl;
        return;
    }

    out << "t " << t << "\n"
        << "size " << positions.size() << "\n";

    for (auto const& position : positions)
        out << "position " << position << "\n";

    for (auto const& speed : speeds)
        out << "speed " << speed << "\n";

    for (auto const& pb : personalBests)
        out << "personalBest_fitness " << pb.second << "\n"
            << "personalBest_position " << pb.first << "\n";

    out << "globalBest_fitness " << globalBest.second << "\n"
        << "globalBest_position " << globalBest.first << "\n";

    out << "absoluteBest_fitness " << absoluteBest.second << "\n"
        << "absoluteBest_position " << absoluteBest.first << "\n";

    out << std::flush;

    if (!out)
    {
        std::cerr << "An error occurred while saving state to file " << filename << std::endl;
        return;
    }

    std::cout << "State successfully saved to file " << filename << std::endl;
}

/*
 * Load saved states
 */
void loadState(std::string const& filename, std::size_t& t, Positions& positions, Speeds& speeds,
               PwFs& personalBests, PositionWithFitness& globalBest,
               PositionWithFitness& absoluteBest)
{
    // In case of error we throw a runtime_error
    std::ifstream in(filename);
    if (!in)
    {
        std::cerr << "Cannot open " << filename << " for reading" << std::endl;
        throw std::runtime_error(filename + " is not readable");
    }

    auto ensureValid = [&in](std::string const& label) {
        if (!in)
        {
            std::cerr << "Couldn't read " << label << std::endl;
            throw std::runtime_error("Couldn't load " + label + " from file");
        }
    };

    std::string trash; // for skipping content
    in >> trash >> t;
    ensureValid("time");

    std::size_t size;
    in >> trash >> size;
    ensureValid("size");

    if (size != positions.size())
    {
        std::cerr << "The state extracted from " << filename << " doesn't match current settings"
                  << std::endl;
        throw std::runtime_error("size mismatch");
    }

    for (std::size_t i = 0; i < size; ++i)
    {
        positions[i].resize(NB_PARAMS); // number of dimensions
        in >> trash >> positions[i];
    }
    ensureValid("positions");

    for (std::size_t i = 0; i < size; ++i)
    {
        speeds[i].resize(NB_PARAMS);
        in >> trash >> speeds[i];
    }
    ensureValid("speeds");

    for (std::size_t i = 0; i < size; ++i)
    {
        personalBests[i].first.resize(NB_PARAMS);
        in >> trash >> personalBests[i].second
           >> trash >> personalBests[i].first;
    }
    ensureValid("personal bests");

    globalBest.first.resize(NB_PARAMS);
    in >> trash >> globalBest.second
       >> trash >> globalBest.first;
    ensureValid("global best");

    absoluteBest.first.resize(NB_PARAMS);
    in >> trash >> absoluteBest.second
       >> trash >> absoluteBest.first;
    ensureValid("absolute best");

}


/*
 * Main function.
 */
int main(int, char const** argv) try
{
    std::string basepath = argv[0];
    auto lastSlashPos = basepath.rfind('/');
    if (lastSlashPos == std::string::npos) {
        basepath = "./";
    } else {
        basepath = basepath.substr(0, lastSlashPos + 1);
    }
    
    std::ofstream fitnessEvolutionFile("fitnessEvolve.csv");
    if(!fitnessEvolutionFile.is_open())
    {
      std::cout << "Error: Cannot open the fitness trace file." << std::endl;
      // throw...
    }

    std::cout << "Basepath is " << basepath << std::endl;

    reset();

    // Read the initial configuration for all robots in order to restore
    // it when measuring the fitness of some PSO settings
    RobotConfigs const initialConfigs = readAllRobotsConfig();

    // Shorthand to compute fitness
    auto computeFitness = [&initialConfigs](Position const& p) {
        // compute performance using Webots
        return simulate(initialConfigs, toParams(p));
    };

    // PSO states:
    Positions positions;               // particles' position
    Speeds speeds;                     // particles' speed
    PwFs personalBests;                // "personal" best for each particle
    PositionWithFitness globalBest;    // "global" best, assuming infinite range of neighbourhood
    PositionWithFitness absoluteBest;  // best of all the particles we have seen so far
    std::size_t t = 0;

    if (isThereFile(basepath + SAVE_FILE))
    {
        loadState(basepath + SAVE_FILE, t, positions, speeds, personalBests, globalBest, absoluteBest);
        std::cout << "Loaded from " << SAVE_FILE << std::endl;
    }
    else
    {
        std::cout << "Randomly initialising PSO state" << std::endl;

        initilisePSO(computeFitness, positions, speeds, personalBests, globalBest, absoluteBest);
        saveState(basepath + SAVE_FILE, t, positions, speeds, personalBests, globalBest, absoluteBest);
    }

    std::cout << "Initial absolute best fitness: " << absoluteBest.second << "\n"
              << "Initial absolute best settings: " << toParams(absoluteBest.first) << std::endl;

    // Run PSO optimisation for a fixed number of iterations
    for (; t < MAX_ITERATIONS; ++t)
    {
        auto sumOfSpeeds = 0.0;

        PositionWithFitness nextGlobalBest;
        nextGlobalBest.second = std::numeric_limits<Fitness>::lowest();

        double averageFitness = 0.0;
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
            averageFitness += fitness;

            // Update our knowledge of the search space
            auto candidate = PositionWithFitness{ positions[i], fitness };
            personalBests[i] = selectBest(personalBests[i], candidate);
            nextGlobalBest = selectBest(nextGlobalBest, candidate);

            std::cout << "Iteration " << t
                      << ", particle " << i
                      << " has fitness " << fitness
                      << std::endl;

            sumOfSpeeds += std::sqrt((speeds[i].apply([](double x) { return x * x; })).sum());
        }
        averageFitness /= positions.size();

        absoluteBest = selectBest(absoluteBest, nextGlobalBest);
        globalBest = nextGlobalBest;

        auto avgOfSpeeds = sumOfSpeeds / positions.size();

        std::cout << "\n\nAVG PSO SPEED: " << avgOfSpeeds << "\n" << std::endl;

        std::cout << "\n\n" << std::string(80, '*') << "\n"
                  << "\tEnd of iteration " << t
                  << " with absolute best fitness of " << absoluteBest.second
                  << " and absolute best settings: " << toParams(absoluteBest.first) << "\n"
                  << " global best has fitness of " << globalBest.second
                  << " and global best settings: " << toParams(globalBest.first) << "\n"
                  << std::string(80, '*') << "\n\n"
                  << std::endl;

        // Save current state to file
        if (isThereFile(basepath + SAVE_FILE))
        {
            // Make a temporary backup just in case
            copyFile(basepath + SAVE_FILE, basepath + SAVE_FILE + ".bak");
        }

        saveState(basepath + SAVE_FILE, t, positions, speeds, personalBests, globalBest, absoluteBest);
        std::cout << "Saved to " << SAVE_FILE << std::endl;
        
        // Update the fitness trace file
        fitnessEvolutionFile << absoluteBest.second << ", " << averageFitness << std::endl;
    }
    
    fitnessEvolutionFile.close();

    std::cout << "Best fitness: " << globalBest.second << "\n"
              << "Best settings: " << toParams(globalBest.first) << std::endl;

    return 0;
} catch (std::runtime_error const& re) {
    std::cerr << "[RUNTIME ERROR] " << re.what() << std::endl;
    return 1;
}

// vim: set spelllang=en_gb
// vim: set ts=4 sw=4
