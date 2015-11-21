#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <webots/robot.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>


/*************************************************** Overall settings     *****/

const int TIME_STEP = 32;


/*************************************************** Robot's capabilities *****/

int robot_id; // Unique robot ID


/*************************************************** Robot's setup        *****/

void reset(void)
{
    char const* robot_name = wb_robot_get_name();
    std::cout << "Reset of robot " << robot_name << std::endl;

    sscanf(robot_name, "epuck%d", &robot_id);
}

/*************************************************** Robot's main algo    *****/

void step()
{
}

int main(int argc, char* args[])
{
    wb_robot_init();
    reset();

    // Loop until webots wants to stop
    while (wb_robot_step(TIME_STEP) != -1) {
        step();
    }

    wb_robot_cleanup();

    return 0;
}

