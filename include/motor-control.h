#include <string>
#include <cmath>
#include <vector>
#include <random>

// --- Global Variables (snake_case) ---
extern bool is_turning;

extern double x_pos, y_pos;
extern double correct_angle;

// --- Function Declarations (lowerCamelCase) ---
void driveChassis(double left_power, double right_power);

double getInertialHeading();
double normalizeTarget(double angle);

void turnToAngle(double turn_angle, double time_limit_msec, bool exit = true, double max_output = 12);
void driveTo(double distance_in, double time_limit_msec, bool exit = true, double max_output = 12);
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit = true, double max_output = 12);

void stopChassis(vex::brakeType type = vex::brake);
void resetChassis();
double getLeftRotationDegree();
double getRightRotationDegree();
void correctHeading();
void trackNoOdomWheel();
void trackXYOdomWheel();
void trackXOdomWheel();
void trackYOdomWheel();
void turnToPoint(double x, double y, int dir, double time_limit_msec);
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);

void resetPositionWithSensor(vex::distance& sensor, double sensor_offset, double sensor_angle_deg, double field_half_size = 72);
void resetPositionFront();
void resetPositionBack();
void resetPositionLeft();
void resetPositionRight();

//MARK: MCL hpp

//A pose is a randomly generated point/particle in space where there is a possiblity the robot exists
//Represented in cordinate form (x-axis position, y-axis position, rotation)
//The struct operation creates a grouping of variables
//In this case, the x, y, and theta cordinates are grouped to improve abstraction and readability
// struct Pose {
//   double x;
//   double y;
//   double theta;
// };

//This class is the overall grouping of headers for MCL functions (methods)
//MCL (Monte Carlo Localization) is an algorithim that improves the accuracy of robot positioning
//MCL creates a multitiude of random points and updates the probability that the robot is at that point
//Probability is determined using sensor data 
// class MCL{
//     //Public functions can be acsessed anywhere
//     public:

//     //Reset the origin of the grouping to a known position (in cordinate form)
//     void reset(double x, double y, double theta);
//     //Adjust the origin pose based on measured robot movement
//     void predict(double dx, double dy, double dtheta);
//     //Update the probability of each pose particle based on sensor data
//     void update(double measuredtheta);
//     //Determin the final best guess of robot position
//     Pose estimate();

//     //Private functions
//     private:

//     //A particle is an educated guess at where the robot is 
//     //Made up of 2 components: the position, and probability that the robot is at said position (weight) 
//     struct Particle
//     {
//         Pose pose;
//         double weight;
//     };
    
//     //A list of all of the particles and their guess properties
//     std::vector<particle> particles;

//     //Discard the particles with a low confidence weighting
//     void resample();
// }
