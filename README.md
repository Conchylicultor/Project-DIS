# Project-DIS
Optimized Simulated Flocking Algorithm for e-pucks


The configuration is defined by two parameters defined in "controlers/common/Config.hpp". All controlers has to be rebuild if one of those change.

* REPLAY: Define if we are in training mode (PSO) or replay mode (just play the best config)
* CROSSING: Define in which world we are (crossing or obstacles)

*Warning*: In training mode, when the program start, it will try to load the previous optimisation state in _controlers/supervisor/results_*.txt_. Delete those files if you want the training to start from scratch.
