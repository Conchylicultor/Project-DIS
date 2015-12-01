#ifndef CONFIG_HPP
#define CONFIG_HPP

/*
 * Here are defined common settings of supervisor and swarm controllers
 */

int const TIME_STEP = 64; // [ms] Length of time step
double const DELTA_T = TIME_STEP/1000.0; // [s] Length of time step

int const EPUCK_EPCUK_CHANNEL = 1;
int const SUPER_EPUCK_CHANNEL = 9;

// Warning: ADAPT TO THE NUMBER OF ROBOTS
int const FLOCK_SIZE = 5;

#endif

