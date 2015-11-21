#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <webots/robot.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>


#define DEBUG_COHESION 1



/*************************************************** Overall settings     *****/

const int TIME_STEP = 32;


/*************************************************** Robot's capabilities *****/

int robot_id; // Unique robot ID
WbDeviceTag emitter;
WbDeviceTag receiver;


/*************************************************** Coordinate system    *****/

// Coordinate for planar space; in Webots it corresponds to X & Z coordinates
struct Point
{
    double x;
    double y;
};

using Points = std::vector<Point>;

Point compute_center(Points const& ps); // invalid if ps is empty!

std::ostream& operator<<(std::ostream& out, Point const& p);

std::ostream& operator<<(std::ostream& out, Points const& ps);

/*************************************************** Robot's setup        *****/

void reset(void)
{
    char const* robot_name = wb_robot_get_name();
    std::cout << "Reset of robot " << robot_name << std::endl;

    sscanf(robot_name, "epuck%d", &robot_id);

    emitter = wb_robot_get_device("emitter");
    receiver = wb_robot_get_device("receiver");

    assert(emitter);
    assert(receiver);
}

void setup_emitter(int channel)
{
    wb_emitter_set_channel(emitter, channel);
    // Other properties are already setup in Webots
}

void setup_receiver(int channel)
{
    wb_receiver_set_channel(receiver, channel);
    wb_receiver_enable(receiver, TIME_STEP);
    // Other properties are already setup in Webots
}


/*************************************************** Robots Cohesion      *****/

// Send a ping in order to let other boids know where this robot is
void ping()
{
    std::string ping = "ping"; // the content doesn't matter as long as it's not empty
    wb_emitter_send(emitter, ping.data(), ping.size());
}

// Find neighbours' relative position
Points find_neighbours()
{
    Points neighbours;

    // Read all packets available; because we have a null-delay this works fine but might be an
    // issue with real-world e-pucks
    for (; wb_receiver_get_queue_length(receiver) > 0; wb_receiver_next_packet(receiver))
    {
        // Compute relative position using the direction and signal = 1/distance^2
        // We assume all robots are on a flat floor and therefore ignore the y-coordinate of the
        // direction
        const double* dir = wb_receiver_get_emitter_direction(receiver);
        double signal = wb_receiver_get_signal_strength(receiver);

        double dirX = dir[0];
        double dirZ = dir[2];
        double theta = std::atan2(dirZ, dirX);

        double distance = std::sqrt(1.0 / signal);

        neighbours.push_back({ std::cos(theta) * distance, std::sin(theta) * distance });
    }

#if defined(DEBUG_COHESION) && DEBUG_COHESION
    std::cout << "Neibours for robot " << robot_id << ": " << neighbours << std::endl;
#endif

    return neighbours;
}


/*************************************************** Robot's main algo    *****/

void step()
{
    // Cohesion: let others know were we are and determine the center of the swarm
    ping();

    Points neighbours = find_neighbours();
    neighbours.push_back({ 0, 0 }); // add our position (local coord system)
    Point const cohesion_center = compute_center(neighbours);

#if defined(DEBUG_COHESION) && DEBUG_COHESION
    std::cout << "Cohesion center for robot " << robot_id << ": " << cohesion_center << std::endl;
#endif
}

int main(int argc, char* args[])
{
    wb_robot_init();
    reset();

    // TODO set channel in Webots controller argument field & read channel from argv here
    int channel = WB_CHANNEL_BROADCAST;

    setup_emitter(channel);
    setup_receiver(channel);

    // Loop until webots wants to stop
    while (wb_robot_step(TIME_STEP) != -1) {
        step();
    }

    wb_robot_cleanup();

    return 0;
}


/*************************************************** Miscellaneous        *****/

Point compute_center(Points const& ps)
{
    assert(ps.size() > 0);

    double sumX = 0, sumY = 0;
    for (auto const& p : ps) {
        sumX += p.x;
        sumY += p.y;
    }

    return { sumX / ps.size(), sumY / ps.size() };
}

std::ostream& operator<<(std::ostream& out, Point const& p)
{
    return out << "{ " << p.x << ", " << p.y << " }";
}

std::ostream& operator<<(std::ostream& out, Points const& ps)
{
    out << "[ ";
    for (auto const& p : ps)
        out << p << " ";
    out << "]";

    return out;
}

