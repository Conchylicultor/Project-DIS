#ifndef UNIFORM_HPP
#define UNIFORM_HPP

#include <mutex>
#include <random>
#include <type_traits>

/*
 * Randomly generate a number on a uniform distribution
 */
template <typename T>
T uniform(T min, T max)
{
    static std::mt19937 algo;

    // Set a random seed only once
    static std::once_flag token;
    std::call_once(token, [] {
        std::random_device rd;
        algo.seed(rd());
    });

    using is_integral = typename std::is_integral<T>;
    using integer_dist = typename std::uniform_int_distribution<T>;
    using real_dist = typename std::uniform_real_distribution<T>;

    using distribution_type = typename std::conditional<is_integral::value, integer_dist, real_dist>::type;

    distribution_type dist(min, max);

    return dist(algo);
}

#endif

// vim: set spelllang=en_gb
// vim: set ts=4 sw=4
