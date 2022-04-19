#include "simulation/velocity_mujoco.hpp"
#include "utilities/math_functions.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

using namespace std;
using namespace pinocchio;
using namespace Eigen;

int main(int argc, char **argv)
{   
    //Ros setting
    ros::init(argc, argv, "kinova_controller");
    ros::NodeHandle n_node;
    ros::Rate loop_rate(1000);

    // Mujoco Subs
    ros::Subscriber jointState = n_node.subscribe("mujoco_ros/mujoco_ros_interface/joint_states", 5, &JointStateCallback, ros::TransportHints().tcpNoDelay(true));\
    ros::Subscriber mujoco_command_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_command_sim2con", 5, &simCommandCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber mujoco_time_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_time", 1, &simTimeCallback, ros::TransportHints().tcpNoDelay(true));

    // Mujoco Pubs
    mujoco_command_pub_ = n_node.advertise<std_msgs::String>("mujoco_ros/mujoco_ros_interface/sim_command_con2sim", 5);
    robot_command_pub_ = n_node.advertise<mujoco_ros_msgs::JointSet>("mujoco_ros/mujoco_ros_interface/joint_set", 5);

    // Mujoco Msg
    robot_command_msg_.position.resize(7); 

    // Ros Param
    string urdf_name, urdf_path;
    n_node.getParam("urdf_path", urdf_path);
    n_node.getParam("urdf_name", urdf_name);

    // Pinocchio
    vector<string> package_dirs;
    package_dirs.push_back(urdf_path);
    std::string urdfFileName = package_dirs[0] + "/" + urdf_name;
    robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, false);  
    model_ = robot_->model();
    Data data(model_);
    data_ = data;

    // Control Variable
    ctrl_mode_ = 0;
    chg_flag_ = false;
    state_.J.resize(6, 7);

    while (ros::ok()){
        keyboard_event();
        
        robot_->computeAllTerms(data_, state_.q, state_.v);
        robot_->jacobianWorld(data_, model_.getJointId("Actuator7"), state_.J);
        state_.x.head(3) = robot_->position(data_, robot_->model().getJointId("Actuator7")).translation();
        state_.rot = robot_->position(data_, robot_->model().getJointId("Actuator7")).rotation();
        state_.xdot.head(3) = robot_->velocity(data_, robot_->model().getJointId("Actuator7")).linear();
        state_.xdot.tail(3) = robot_->velocity(data_, robot_->model().getJointId("Actuator7")).angular();

        if (ctrl_mode_== 0){
            state_.q_des.setZero();
        }
        if (ctrl_mode_ ==1){ // home
            if (chg_flag_){
                cubic_.stime = time_;
                cubic_.ftime = time_+ 2.0;
                cubic_.q0 = state_.q;
                cubic_.v0 = state_.v;
                
                q_target_(0) = 0. / 180. * M_PI;
                q_target_(1) = -40.0 / 180. * M_PI;
                q_target_(2) = 180. / 180. * M_PI;
                q_target_(3) = -100 / 180. * M_PI;
                q_target_(4) = 0. / 180. * M_PI;
                q_target_(5) = 60. / 180. * M_PI;
                q_target_(6) = 90. / 180. * M_PI;

                chg_flag_ = false;
            }
            for (int i=0; i<7; i++)
                state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), cubic_.v0(i), 0.0);
        }
        if (ctrl_mode_ ==2){ // Move Forward
            if (chg_flag_){
                cubic_.stime = time_;
                cubic_.ftime = time_+ 3.0;
                cubic_.x0 = state_.x;
                // cubic_.v0 = state_.v;
                
                // q_target_(0) = 0. / 180. * M_PI;
                // q_target_(1) = -40.0 / 180. * M_PI;
                // q_target_(2) = 180. / 180. * M_PI;
                // q_target_(3) = -100 / 180. * M_PI;
                // q_target_(4) = 0. / 180. * M_PI;
                // q_target_(5) = 60. / 180. * M_PI;
                // q_target_(6) = 90. / 180. * M_PI;

                chg_flag_ = false;
            }
            // for (int i=0; i<7; i++)
            //     state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), cubic_.v0(i), 0.0);
        }

        if (ctrl_mode_ ==999){ // off position
            if (chg_flag_){
                cubic_.stime = time_;
                cubic_.ftime = time_+ 2.0;
                cubic_.q0 = state_.q;
                cubic_.v0 = state_.v;
                
                q_target_(0) = 0. / 180. * M_PI;
                q_target_(1) = -120 / 180. * M_PI;
                q_target_(2) = 180. / 180. * M_PI;
                q_target_(3) = -140 / 180. * M_PI;
                q_target_(4) = 0. / 180. * M_PI;
                q_target_(5) = 20. / 180. * M_PI;
                q_target_(6) = 90. / 180. * M_PI;

                chg_flag_ = false;
            }
            for (int i=0; i<7; i++)
                state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), cubic_.v0(i), 0.0);
        }

        //cout << "Position" << robot_->position(data_, robot_->model().getJointId("Actuator7")) << endl;
        
        robot_command_msg_.MODE = 0;
        robot_command_msg_.header.stamp = ros::Time::now();
        robot_command_msg_.time = time_;
        for (int i=0; i<7; i++)
            robot_command_msg_.position[i] = state_.q_des(i);
        robot_command_pub_.publish(robot_command_msg_);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


void simCommandCallback(const std_msgs::StringConstPtr &msg){
    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_command_pub_.publish(rst_msg_);
        mujoco_time_ = 0.0;
    }
}

void simTimeCallback(const std_msgs::Float32ConstPtr &msg){
    mujoco_time_ = msg->data;
    time_ = mujoco_time_;
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
  for (int i=0; i<7; i++){
    state_.q(i) = msg->position[i];
    state_.v(i) = msg->velocity[i];
  }
}

void keyboard_event(){
    if (_kbhit()){
        int key;
        key = getchar();
        int msg = 0;
        switch (key){
            case 'h': //home
                ctrl_mode_ = 1;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move to Home Position" << endl;
                cout << " " << endl;
                break;
            case 'a': //down
                ctrl_mode_ = 2;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move Down" << endl;
                cout << " " << endl;
                break;
            case 'q': //quit
                ctrl_mode_= 999;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move to Off Position" << endl;
                cout << " " << endl;
                break;    
        }
    }
}
