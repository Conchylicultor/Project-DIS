#include <iostream>
#include <vector>
#include <cstdio> // For sscanf
#include <cmath>
#include <string>

#include <webots/robot.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

using namespace std;


// Global parametters
static const int FLOCK_SIZE = 5; // Number of robot in each flock (WARNING: Adapt to the number of robot)
static const int TIME_STEP = 64; // [ms] Length of time step
static const float DELTA_T = TIME_STEP/1000.0f; // [ms] Length of time step

string robotName;
int robotIdGlobal = 0; // World id
int flockId = 0; // Id of the flock
int robotId = 0; // Id in the flock

static const int NB_SENSORS = 8; // Number of distance sensors

WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node


// -------------------
// Coordinate variables
// -------------------

/*
 * Coordinate for planar space; in Webots it corresponds to X & Z coordinates
 */
struct Vec2
{
    Vec2() : x(0.0f), y(0.0f) {}
    Vec2(float argX, float argY) : x(argX), y(argY) {}
    
    Vec2& operator=(const Vec2& other) // copy assignment
    {
        x = other.x;
        y = other.y;
        return *this;
    }
    
    Vec2 operator-(const Vec2& other) // copy assignment
    {
        return Vec2(x - other.x,
                    y - other.y);
    }
    
    Vec2 operator/(float factor) // copy assignment
    {
        return Vec2(x/factor,
                    y/factor);
    }
    
    float x;
    float y;
};

std::ostream& operator<<(std::ostream& out, Vec2 const& p)
{
    return out << "{ " << p.x << ", " << p.y << " }";
}

Vec2 myPosition(0.0f,0.0f);
Vec2 myPrevPosition(0.0f,0.0f);
float myTheta = 0.0f;

Vec2 neighboursPos[FLOCK_SIZE];
Vec2 neighboursPrevPos[FLOCK_SIZE];
Vec2 neighboursRelativeSpeed[FLOCK_SIZE];

// -------------------
// Obstacle avoidance functions
// -------------------

/*
 * Try avoiding obstacles
 */
void braitenbergObstacle(int wheelSpeed[2])
{
  // Parametters
  static int MIN_SENS = 350;
  static int weightMatrix[2][NB_SENSORS] = {{-72,-58,-36,8,10,36,28,18},
                                            {18,28,36,10,8,-36,-58,-72}}; // Braitenberg weight
  int distance = 0;
  
  // Reinitialisation
  wheelSpeed[0] = 0;
  wheelSpeed[1] = 0;
  
  // Compute the ponderate behavior
  for(int i=0 ; i<NB_SENSORS ; i++)
  {
    distance = wb_distance_sensor_get_value(ds[i]); //Read sensor values
    
    // Weighted sum of distance sensor values for Braitenburg vehicle
    wheelSpeed[0] += weightMatrix[0][i] * distance; // Left
    wheelSpeed[1] += weightMatrix[1][i] * distance; // Right
  }
  
  // Adapt the speed
  wheelSpeed[0] /= MIN_SENS;
  wheelSpeed[1] /= MIN_SENS;
}

// -------------------
// Communication functions
// -------------------

/*
 * Send a ping in order to let other boids know where this robot is
 */
void ping()
{
  string message = std::to_string(flockId) + "_" + std::to_string(robotId); // Send our id (and flock id)
  wb_emitter_send(emitter, message.c_str(), message.size()+1);
}

/*
 * Receive messages from other and extract interresting informations
 */
void pong()
{
  // Read all packets available; because we have a null-delay this works fine
  // but might be an issue with real-world e-pucks
  for (; wb_receiver_get_queue_length(receiver) > 0; wb_receiver_next_packet(receiver))
  {
    // Read received data
    string receivedMessage = static_cast<const char*>(wb_receiver_get_data(receiver));
    const double* dir      = wb_receiver_get_emitter_direction(receiver);
    double signalStrength  = wb_receiver_get_signal_strength(receiver);
    
    // Compute and extract the robot id
    int receivedRobotId;
    int receivedFlockId;
    sscanf(receivedMessage.c_str(),"%d_%d", &receivedFlockId, &receivedRobotId);
    
    if(receivedFlockId != flockId) // We ignore robots from other flock
    {
      continue;
    }
    
    // Compute the position & cie
    float dirX = dir[0]; // WHY ON REYNOLD2.C IS X EQUAL TO DIR[1] ???
    float dirZ = dir[2];
    
    float theta = -std::atan2(dirZ, dirX);
    theta += myTheta; // Relative orientation of our neighbour
    
    float distance = std::sqrt(1.0 / signalStrength);
    
    neighboursPrevPos[receivedRobotId] = neighboursPos[receivedRobotId];
    
    neighboursPos[receivedRobotId] = { std::cos(theta) * distance, 
                                       -std::sin(theta) * distance };

    neighboursRelativeSpeed[receivedRobotId] = (neighboursPos[receivedRobotId] - neighboursPrevPos[receivedRobotId])/DELTA_T;
    
    //cout << "-----------------------------------" << endl;
    //cout << receivedFlockId << " " << receivedRobotId << " : " << endl;
    //cout << neighboursPrevPos[receivedRobotId] << endl;
    //cout << neighboursPos[receivedRobotId] << endl;
    //cout << neighboursRelativeSpeed[receivedRobotId] << endl;
  }
}

// -------------------
// Main functions
// -------------------

/*
 * Reset the robot's devices and get its ID
 */
static void reset()
{
    wb_robot_init();
    
    // Loading robot name
    robotName = wb_robot_get_name();
    sscanf(robotName.c_str(),"epuck%d",&robotIdGlobal); // read robot id from the robot's name
    robotId = robotIdGlobal % FLOCK_SIZE; // normalize between 0 and FLOCK_SIZE-1
    flockId = (robotIdGlobal/FLOCK_SIZE) + 1;
    
    cout << "Loading: robot " << robotIdGlobal << " (" << robotId << " in flock " << flockId << ")" << endl;
    
    // Loading R&B module
    receiver = wb_robot_get_device("receiver");
    emitter = wb_robot_get_device("emitter");
    wb_receiver_enable(receiver,64);
    
    // Loading the distances sensors
    for(int i=0; i<NB_SENSORS;i++) {
        ds[i]=wb_robot_get_device(string("ps" + std::to_string(i)).c_str());	// the device name is specified in the world file
        wb_distance_sensor_enable(ds[i],64);
    }
}

/*
 * Main function.
 */
int main(){ 
  // Initialize the robot
  reset();
  
  int wheelSpeed[2] = {0,0}; // Left and right wheel speed
  int wheelSpeedBraitenberg[2] = {0,0}; // Left and right wheel speed
  
  // Main loop
  while(true)
  {
    // Reinitialization
    
    // Braitenberg obstacle avoidance
    braitenbergObstacle(wheelSpeedBraitenberg);
    
    // Emission/reception between flock members
    ping(); // Indicate our presence
    pong(); // Get informations from other robots
    
    // Compute speed
    wheelSpeed[0] = 300;
    wheelSpeed[1] = 300;
    
    // Add Braitenberg
    wheelSpeed[0] += wheelSpeedBraitenberg[0];
    wheelSpeed[1] += wheelSpeedBraitenberg[1];
    
    // Set speed
    wb_differential_wheels_set_speed(wheelSpeed[0],wheelSpeed[1]);
    
    // Continue one step
    wb_robot_step(TIME_STEP);
  }
}  

