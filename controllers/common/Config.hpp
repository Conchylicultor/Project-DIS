#ifndef CONFIG_HPP
#define CONFIG_HPP

/*
 * Here are defined common settings of supervisor and swarm controllers
 */

#define REPLAY 1 // if 1, only send best settings from PSO to e-pucks
#define CROSSING 0 // 1 if crossing world, 0 otherwise (!!! REBUILD BOTH CONTROLERS !!!!)



int const TIME_STEP = 64; // [ms] Length of time step
double const DELTA_T = TIME_STEP/1000.0; // [s] Length of time step

int const EPUCK_EPCUK_CHANNEL = 1;
int const SUPER_EPUCK_CHANNEL = 9;

// Warning: ADAPT TO THE NUMBER OF ROBOTS
#if CROSSING == 1
  int const NUMBER_OF_FLOCKS = 2; // Crossing: 2 flock
#else
  int const NUMBER_OF_FLOCKS = 1;
#endif

int const FLOCK_SIZE = 5;


#endif

