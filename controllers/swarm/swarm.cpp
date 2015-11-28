
#include <cmath>
#include <cstdio>
#include <cstring>

#include <webots/robot.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

using namespace std;

const int NB_SENSORS = 8;              // Number of distance sensors
const int MIN_SENS = 350;              // Minimum sensibility value
const int MAX_SENS = 4096;             // Maximum sensibility value
const int MAX_SPEED = 800;             // Maximum speed
const int FLOCK_SIZE = 5;              // Size of flock
const int TIME_STEP = 64;              // [ms] Length of time step

const double AXLE_LENGTH = 0.052;          // Distance between wheels of robot (meters)
const double SPEED_UNIT_RADS = 0.00628;    // Conversion factor from speed unit to radian per second
const double WHEEL_RADIUS = 0.0205;        // Wheel radius (meters)
const double DELTA_T = TIME_STEP / 1000.0; // Timestep (seconds)

const double RULE1_THRESHOLD = 0.2;      // Threshold to activate aggregation rule. default 0.20
const double RULE1_WEIGHT = 0.2;         // Weight of aggregation rule. default 0.20

const double RULE2_THRESHOLD = 0.1;      // Threshold to activate dispersion rule. default 0.1
const double RULE2_WEIGHT = 1.0;         // Weight of dispersion rule. default 1.0

const double RULE3_WEIGHT = 0.01; // Weight of consistency rule. default 0.01

const double MIGRATION_WEIGHT = 0.01; // Weight of attraction towards the common goal. default 0.01

// Braitenberg parameters for obstacle avoidance
int e_puck_matrix[16] = {
    17, 29, 34, 10, 8, -38, -56, -76, -72, -58, -36, 8, 10, 36, 28, 18
}; // for obstacle avoidance

WbDeviceTag ds[NB_SENSORS];    // Handle for the infrared distance sensors
WbDeviceTag receiver;          // Handle for the receiver node
WbDeviceTag emitter;           // Handle for the emitter node

int robot_id_u, robot_id; // Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID

double relative_pos[FLOCK_SIZE][3];      // relative X, Z, Theta of all robots
double prev_relative_pos[FLOCK_SIZE][3]; // Previous relative  X, Z, Theta values
double my_position[3];                   // X, Z, Theta of the current robot
double prev_my_position[3];           // X, Z, Theta of the current robot in the previous time step
double speed[FLOCK_SIZE][2];          // Speeds calculated with Reynold's rules
double relative_speed[FLOCK_SIZE][2]; // Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];          // != 0 if initial positions have been received
double migr[2] = { 25, -25 };         // Migration vector
char const* robot_name;

/*
 * Reset the robot's devices and get its ID
 */
static void reset()
{
    wb_robot_init();

    receiver = wb_robot_get_device("receiver");
    emitter = wb_robot_get_device("emitter");

    int i;
    char s[4] = "ps0";
    for (i = 0; i < NB_SENSORS; i++)
    {
        ds[i] = wb_robot_get_device(s);         // the device name is specified in the world file
        s[2]++;                                 // increases the device number
    }
    robot_name = static_cast<char const*>(wb_robot_get_name());

    for (i = 0; i < NB_SENSORS; i++)
        wb_distance_sensor_enable(ds[i], 64);

    wb_receiver_enable(receiver, 64);

    // Reading the robot's name. Pay attention to name specification when adding robots to the
    // simulation!
    sscanf(robot_name, "epuck%d", &robot_id_u);   // read robot id from the robot's name
    robot_id = robot_id_u % FLOCK_SIZE;           // normalize between 0 and FLOCK_SIZE-1

    for (i = 0; i < FLOCK_SIZE; i++)
    {
        initialized[i] = 0; // Set initialization to 0 (= not yet initialized)
    }

    printf("Reset: robot %d\n", robot_id_u);
}

/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int* number, int limit)
{
    if (*number > limit)
        *number = limit;
    if (*number < -limit)
        *number = -limit;
}

/*
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr)
{
    double theta = my_position[2];

    // Compute deltas of the robot
    double dr = msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double dl = msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double du = (dr + dl) / 2.0;
    double dtheta = (dr - dl) / AXLE_LENGTH;

    // Compute deltas in the environment
    double dx = -du * sin(theta);
    double dz = -du * cos(theta);

    // Update position
    my_position[0] += dx;
    my_position[1] += dz;
    my_position[2] += dtheta;

    // Keep orientation within 0, 2pi
    if (my_position[2] > 2 * M_PI)
        my_position[2] -= 2.0 * M_PI;
    if (my_position[2] < 0)
        my_position[2] += 2.0 * M_PI;
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int* msl, int* msr)
{
    // Compute wanted position from Reynold's speed and current location
    double x = speed[robot_id][0] * cos(my_position[2]) -
               speed[robot_id][1] * sin(my_position[2]); // x in robot coordinates
    double z = -speed[robot_id][0] * sin(my_position[2]) -
               speed[robot_id][1] * cos(my_position[2]); // z in robot coordinates

    double Ku = 0.2;                     // Forward control coefficient
    double Kw = 10.0;                    // Rotational control coefficient
    double range = sqrt(x * x + z * z);  // Distance to the wanted position
    double bearing = -atan2(x, z);       // Orientation of the wanted position

    // Compute forward control
    double u = Ku * range * cos(bearing);
    // Compute rotational control
    double w = Kw * range * sin(bearing);

    // Convert to wheel speeds!
    *msl = 50 * (u - AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
    *msr = 50 * (u + AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
    limit(msl, MAX_SPEED);
    limit(msr, MAX_SPEED);
}

/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules()
{
    int i, j, k;                         // Loop counters
    double rel_avg_loc[2] = { 0, 0 };    // Flock average positions
    double rel_avg_speed[2] = { 0, 0 };  // Flock average speeds
    double cohesion[2] = { 0, 0 };
    double dispersion[2] = { 0, 0 };
    double consistency[2] = { 0, 0 };

    /* Compute averages over the whole flock */
    for (j = 0; j < 2; j++)
    {
        for (i = 0; i < FLOCK_SIZE; i++)
        {
            // don't consider yourself for the average
            if (i != robot_id)
            {
                rel_avg_speed[j] += relative_speed[i][j];
                rel_avg_loc[j] += relative_pos[i][j];
            }
        }
    }

    /* Rule 1 - Aggregation/Cohesion: move towards the center of mass */

    for (j = 0; j < 2; j++)
    {
        // If center of mass is too far
        if (abs(rel_avg_loc[j]) > RULE1_THRESHOLD)
        {
            cohesion[j] = rel_avg_loc[j]; // Relative distance to the center of the swarm
        }
    }

    /* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
    for (k = 0; k < FLOCK_SIZE; k++)
    {
        if (k != robot_id)
        { // Loop on flockmates only
            // If neighbor k is too close
            if (pow(relative_pos[k][0], 2) + pow(relative_pos[k][1], 2) < RULE2_THRESHOLD)
            {
                for (j = 0; j < 2; j++)
                {
                    dispersion[j] -= relative_pos[k][j]; // Relative distance to k
                }
            }
        }
    }

    /* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
    for (j = 0; j < 2; j++)
    {
        consistency[j] = rel_avg_speed[j]; // difference speed to the average
    }

    // aggregation of all behaviors with relative influence determined by weights
    for (j = 0; j < 2; j++)
    {
        speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
        speed[robot_id][j] += dispersion[j] * RULE2_WEIGHT;
        speed[robot_id][j] += consistency[j] * RULE3_WEIGHT;
        speed[robot_id][j] += (migr[j] - my_position[j]) * MIGRATION_WEIGHT;
    }
}

/*
 * Each robot sends a ping message, so the other robots can measure relative range and bearing to
 * the sender.
 * The message contains the robot's name
 * The range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void)
{
    char out[10];
    strcpy(out, robot_name); // in the ping message we send the name of the robot.
    wb_emitter_send(emitter, out, strlen(out));
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void)
{
    const double* message_direction;
    double message_rssi; // Received Signal Strength indicator
    double theta;
    double range;
    char* inbuffer; // Buffer for the receiver node
    int other_robot_id;
    while (wb_receiver_get_queue_length(receiver) > 0)
    {
        inbuffer = (char*)wb_receiver_get_data(receiver);
        message_direction = wb_receiver_get_emitter_direction(receiver);
        message_rssi = wb_receiver_get_signal_strength(receiver);
        double y = message_direction[2];
        double x = message_direction[0];

        theta = -atan2(y, x);
        theta += my_position[2]; // find the relative theta;
        range = sqrt((1 / message_rssi));

        other_robot_id = (int)(inbuffer[5] - '0'); // since the name of the sender is in the
                                                   // received message. Note: this does not work for
                                                   // robots having id bigger than 9!

        // Get position update
        prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
        prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

        relative_pos[other_robot_id][0] = range * cos(theta);          // relative x pos
        relative_pos[other_robot_id][1] = -range * sin(theta);         // relative y pos

        // printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my
        // theta%g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],my_position[2]*180.0/3.141592,my_position[2]*180.0/3.141592);

        relative_speed[other_robot_id][0] = (1 / DELTA_T) * (relative_pos[other_robot_id][0] -
                                                             prev_relative_pos[other_robot_id][0]);
        relative_speed[other_robot_id][1] = (1 / DELTA_T) * (relative_pos[other_robot_id][1] -
                                                             prev_relative_pos[other_robot_id][1]);
        wb_receiver_next_packet(receiver);
    }
}

// the main function
int main()
{
    int msl, msr;                       // Wheel speeds
    int bmsl, bmsr, sum_sensors;        // Braitenberg parameters
    int i;                              // Loop counter
    int distances[NB_SENSORS];          // Array for the distance sensor readings
    int max_sens;                       // Store highest sensor value

    reset(); // Resetting the robot

    msl = 0;
    msr = 0;
    max_sens = 0;

    // Forever
    for (;;)
    {

        bmsl = 0;
        bmsr = 0;
        sum_sensors = 0;
        max_sens = 0;

        /* Braitenberg */
        for (i = 0; i < NB_SENSORS; i++)
        {
            distances[i] = wb_distance_sensor_get_value(ds[i]); // Read sensor values
            sum_sensors += distances[i];                        // Add up sensor values
            max_sens = max_sens > distances[i] ? max_sens
                                               : distances[i]; // Check if new highest sensor value

            // Weighted sum of distance sensor values for Braitenburg vehicle
            bmsr += e_puck_matrix[i] * distances[i];
            bmsl += e_puck_matrix[i + NB_SENSORS] * distances[i];
        }

        // Adapt Braitenberg values (empirical tests)
        bmsl /= MIN_SENS;
        bmsr /= MIN_SENS;
        //                   bmsl+=66; bmsr+=72;

        /* Send and get information */

        send_ping(); // sending a ping to other robot, so they can measure their distance to this
                     // robot

        process_received_ping_messages();

        // Compute self position
        prev_my_position[0] = my_position[0];
        prev_my_position[1] = my_position[1];

        update_self_motion(msl, msr);

        speed[robot_id][0] = (1 / DELTA_T) * (my_position[0] - prev_my_position[0]);
        speed[robot_id][1] = (1 / DELTA_T) * (my_position[1] - prev_my_position[1]);

        // Reynold's rules with all previous info (updates the speed[][] table)
        reynolds_rules();

        // Compute wheels speed from reynold's speed
        compute_wheel_speeds(&msl, &msr);

        // Adapt speed instinct to distance sensor values
        if (sum_sensors > NB_SENSORS * MIN_SENS)
        {
            msl -= msl * max_sens / (2 * MAX_SENS);
            msr -= msr * max_sens / (2 * MAX_SENS);
        }

        // Add Braitenberg
        msl += bmsl;
        msr += bmsr;

        // Set speed
        wb_differential_wheels_set_speed(msl, msr);

        // Continue one step
        wb_robot_step(TIME_STEP);
    }
}

