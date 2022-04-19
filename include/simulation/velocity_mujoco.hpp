//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include "utilities/urdf_to_pin.hpp"

//Mujoco MSG Header
#include "mujoco_ros_msgs/JointSet.h"
#include "mujoco_ros_msgs/SensorState.h"

// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"

//SYSTEM Header
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;
using namespace pinocchio;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

typedef pinocchio::Model Model;
typedef pinocchio::Data Data;

typedef struct State {   
    Vector7d q;
    Vector7d v;
    Vector6d x;
    Vector6d xdot;
    Vector7d q_des;
    Matrix3d rot;
    Data::Matrix6x J;
} state;  
typedef struct CubicVar {
    Vector7d q0;
    Vector7d v0;
    Vector6d x0;
    double stime;
    double ftime;
} cubicvar;
/////////////////////////////////////////////////////
// Ros Pub
ros::Publisher mujoco_command_pub_;
ros::Publisher robot_command_pub_;

// Ros Msg
mujoco_ros_msgs::JointSet robot_command_msg_;

// Pinocchio
std::shared_ptr<RobotWrapper> robot_;
Model model_;
Data data_;

// Control Variable
double mujoco_time_, time_;
state state_;
cubicvar cubic_;
int ctrl_mode_;
bool chg_flag_;
Vector7d q_target_;
Vector6d x_target_;

// Callback Function
void simCommandCallback(const std_msgs::StringConstPtr &msg);
void simTimeCallback(const std_msgs::Float32ConstPtr &msg);
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

// Keyboard Function
void keyboard_event();
bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
};