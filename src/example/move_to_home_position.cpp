#include "example/move_to_home_position.hpp"
#include "utilities/math_functions.hpp"

using namespace std;
using namespace k_api;

int main(int argc, char **argv)
{
    //Ros setting
    ros::init(argc, argv, "kinova_controller");
    ros::NodeHandle n_node;
    ros::Rate loop_rate(1000);
    
    int PORT = 10000;
    string ip_address = "192.168.1.10";
    string username = "admin";
    string password = "admin";  

    ctrl_flag_ = 0;
    COMMAND_SUCCEESS_ = true;

    string urdf_name, urdf_path;
    n_node.getParam("/kimm_kortex_custum_home/port_number", PORT);
    n_node.getParam("/kimm_kortex_custum_home/ip_address", ip_address);
    n_node.getParam("/kimm_kortex_custum_home/username", username);
    n_node.getParam("/kimm_kortex_custum_home/password", password);
    n_node.getParam("urdf_path", urdf_path);
    n_node.getParam("urdf_name", urdf_name);
    n_node.getParam("iskinovaonly", iskinovaonly_);

    // Pinocchio Model
    vector<string> package_dirs;
    package_dirs.push_back(urdf_path);
    std::string urdfFileName = package_dirs[0] + "/" + urdf_name;
    if (iskinovaonly_){
        pinocchio::urdf::buildModel(urdfFileName, model_, false);
        Data data(model_);
        data_ = data;

        q_.setZero(7);
        v_.setZero(7);
        J_.setZero(6, 7);
        x_.setZero();
    
        joint_state_publisher_ = n_node.advertise<sensor_msgs::JointState>("arm/joint_states", 100);   
        joint_msg_.name.resize(7);
        joint_msg_.position.resize(7);

        std::vector<std::string> joint_names;
        joint_names.push_back("joint_1");
        joint_names.push_back("joint_2");
        joint_names.push_back("joint_3");
        joint_names.push_back("joint_4");
        joint_names.push_back("joint_5");
        joint_names.push_back("joint_6");
        joint_names.push_back("joint_7");
        joint_msg_.name = joint_names;
    }
    else{
        pinocchio::urdf::buildModel(urdfFileName, pinocchio::JointModelFreeFlyer(), model_, false);
        Data data(model_);
        data_ = data;

        q_.setZero(26);
        v_.setZero(25);
        J_.setZero(6, 7);
        x_.setZero();
    
        joint_state_publisher_ = n_node.advertise<sensor_msgs::JointState>("arm/joint_states", 1);
        joint_msg_.name.resize(7);
        joint_msg_.position.resize(7);

        std::vector<std::string> joint_names;
        joint_names.push_back("joint_1");
        joint_names.push_back("joint_2");
        joint_names.push_back("joint_3");
        joint_names.push_back("joint_4");
        joint_names.push_back("joint_5");
        joint_names.push_back("joint_6");
        joint_names.push_back("joint_7");
        joint_msg_.name = joint_names;

        body_state_subscriber_ = n_node.subscribe("/spot/odometry", 1, &bodyStateCallback_);
    }
    
    //Kinova Setting
    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(ip_address, PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(username);
    create_session_info.set_password(password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);

    

    while (ros::ok()){
        keyboard_event();

        joint_from_kinova_ = base->GetMeasuredJointAngles();
        GetJointState();

        pinocchio::computeAllTerms(model_, data_, q_, v_);
                
        if (ctrl_flag_ == 1 && !COMMAND_SUCCEESS_){
            COMMAND_SUCCEESS_ = true;

            auto servoingMode = k_api::Base::ServoingModeInformation();
            servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
            base->SetServoingMode(servoingMode);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Move arm to ready position
            std::cout << "Moving the arm to a safe position" << std::endl;
            auto action_type = k_api::Base::RequestedActionType();
            action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
            auto action_list = base->ReadAllActions(action_type);
            auto action_handle = k_api::Base::ActionHandle();
            action_handle.set_identifier(0);
            for (auto action : action_list.action_list()) 
            {
                if (action.name() == "NewHome") 
                {
                    action_handle = action.handle();
                }
            }

            if (action_handle.identifier() == 0) 
            {
                std::cout << "Can't reach safe position, exiting" << std::endl;
                return false;
            } 
            else 
            {
                // Connect to notification action topic
                std::promise<k_api::Base::ActionEvent> finish_promise;
                auto finish_future = finish_promise.get_future();
                auto promise_notification_handle = base->OnNotificationActionTopic(
                    create_event_listener_by_promise(finish_promise),
                    k_api::Common::NotificationOptions()
                );

                // Execute action
                base->ExecuteActionFromReference(action_handle);

                // Wait for future value from promise
                const auto status = finish_future.wait_for(std::chrono::seconds{20});
                base->Unsubscribe(promise_notification_handle);

                if(status != std::future_status::ready)
                {
                    std::cout << "Timeout on action notification wait" << std::endl;
                }
                const auto promise_event = finish_future.get();

                std::cout << "Move to Home completed" << std::endl;
                std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl; 
            }
        } // Home Position CTRL

        if (ctrl_flag_ == 2){
            if (!COMMAND_SUCCEESS_){
                oMi_ = data_.oMi[model_.getJointId("joint_7")];
                ROS_WARN_STREAM(oMi_);
                COMMAND_SUCCEESS_ = true;
            }

            if (iskinovaonly_){
                Data::Matrix6x J_tmp(6, 7);
                pinocchio::getJointJacobian(model_, data_, model_.getJointId("joint_7"), pinocchio::LOCAL, J_tmp);
                J_ = J_tmp;
                
                SE3 M_ref = oMi_;
                M_ref.translation()(0) += 0.1;
                SE3 oMi = data_.oMi[model_.getJointId("joint_7")];
                SE3 wMl;
                wMl.setIdentity();
                Motion v_frame = data_.v[model_.getJointId("joint_7")];
                Motion p_error, v_ref, v_error;
                VectorXd p_ref, p, p_error_vec, v_error_vec, v;
                p_ref.setZero(12);
                p.setZero(12);

                errorInSE3(oMi, M_ref, p_error);
                SE3ToVector(M_ref, p_ref);
                SE3ToVector(oMi, p);
                
                wMl.rotation(oMi.rotation());
                p_error_vec = p_error.toVector();
                v_error =  wMl.actInv(v_ref) - v_frame;
                
                VectorXd xdot_des = p_error_vec * 2.;// + v_error.toVector(); 

                Eigen::MatrixXd J_inv = J_.completeOrthogonalDecomposition().pseudoInverse();
                VectorXd v_arm_des = J_inv * xdot_des;
                
                k_api::Base::JointSpeeds joint_speeds;
                for (size_t i = 0 ; i < 7; ++i)
                {
                    auto joint_speed = joint_speeds.add_joint_speeds();
                    joint_speed->set_joint_identifier(i);
                    joint_speed->set_value(v_arm_des(i) * 180.0 /M_PI);
                    joint_speed->set_duration(0.0001);
                }
                base->SendJointSpeedsCommand(joint_speeds);
            }
            else{
                Data::Matrix6x J_tmp(6, 25);
                pinocchio::getJointJacobian(model_, data_, model_.getJointId("joint_7"), pinocchio::LOCAL, J_tmp);
                J_ = J_tmp.block(0, 6, 6, 7);
                
                SE3 M_ref = oMi_;
                // M_ref.translation()(0) += 0.0;
                SE3 oMi = data_.oMi[model_.getJointId("joint_7")];
                SE3 wMl;
                wMl.setIdentity();
                Motion v_frame = data_.v[model_.getJointId("joint_7")];
                Motion p_error, v_ref, v_error;
                VectorXd p_ref, p, p_error_vec, v_error_vec, v;
                p_ref.setZero(12);
                p.setZero(12);

                errorInSE3(oMi, M_ref, p_error);
                SE3ToVector(M_ref, p_ref);
                SE3ToVector(oMi, p);
                
                wMl.rotation(oMi.rotation());
                p_error_vec = p_error.toVector();
                v_error =  wMl.actInv(v_ref) - v_frame;
                
                VectorXd xdot_des = p_error_vec * 2.0;// + v_error.toVector(); 

                Eigen::MatrixXd J_inv = J_.completeOrthogonalDecomposition().pseudoInverse();
                VectorXd v_arm_des = J_inv * xdot_des;

                k_api::Base::JointSpeeds joint_speeds;
                for (size_t i = 0 ; i < 7; ++i)
                {
                    auto joint_speed = joint_speeds.add_joint_speeds();
                    joint_speed->set_joint_identifier(i);
                    joint_speed->set_value(v_arm_des(i) * 180.0 /M_PI);
                    joint_speed->set_duration(0.0001);
                }
                base->SendJointSpeedsCommand(joint_speeds);
            }
            
        } // Velocity Test

        if (ctrl_flag_ == 999 && !COMMAND_SUCCEESS_){
            COMMAND_SUCCEESS_ = true;

            auto servoingMode = k_api::Base::ServoingModeInformation();
            servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
            base->SetServoingMode(servoingMode);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Move arm to ready position
            std::cout << "Moving the arm to a safe position" << std::endl;
            auto action_type = k_api::Base::RequestedActionType();
            action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
            auto action_list = base->ReadAllActions(action_type);
            auto action_handle = k_api::Base::ActionHandle();
            action_handle.set_identifier(0);
            for (auto action : action_list.action_list()) 
            {
                if (action.name() == "NewTurnOff") 
                {
                    action_handle = action.handle();
                }
            }

            if (action_handle.identifier() == 0) 
            {
                std::cout << "Can't reach safe position, exiting" << std::endl;
                return false;
            } 
            else 
            {
                // Connect to notification action topic
                std::promise<k_api::Base::ActionEvent> finish_promise;
                auto finish_future = finish_promise.get_future();
                auto promise_notification_handle = base->OnNotificationActionTopic(
                    create_event_listener_by_promise(finish_promise),
                    k_api::Common::NotificationOptions()
                );

                // Execute action
                base->ExecuteActionFromReference(action_handle);

                // Wait for future value from promise
                const auto status = finish_future.wait_for(std::chrono::seconds{20});
                base->Unsubscribe(promise_notification_handle);

                if(status != std::future_status::ready)
                {
                    std::cout << "Timeout on action notification wait" << std::endl;
                }
                const auto promise_event = finish_future.get();

                std::cout << "Move to Safe Position completed" << std::endl;
                std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl; 
            }
        } // Quit

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void GetJointState(){
    joint_msg_.header.stamp = ros::Time::now();
    
    int i = 0;
    for (auto joint_angle : joint_from_kinova_.joint_angles()) 
    {
        joint_msg_.position[i] = joint_angle.value() * M_PI / 180.0;
        if (iskinovaonly_)
            q_(i) = joint_msg_.position[i];
        else
            q_(i+7) = joint_msg_.position[i];
        i++;
    }
    joint_state_publisher_.publish(joint_msg_);
    
}
void bodyStateCallback_ (const nav_msgs::Odometry::ConstPtr& msg){
    
    q_(0) = msg->pose.pose.position.x;
    q_(1) = msg->pose.pose.position.y;
    q_(2) = msg->pose.pose.position.z;

    q_(3) = msg->pose.pose.orientation.x;
    q_(4) = msg->pose.pose.orientation.y;
    q_(5) = msg->pose.pose.orientation.z;
    q_(6) = msg->pose.pose.orientation.w;
}

void keyboard_event(){
    if (_kbhit()){
        int key;
        key = getchar();
        int msg = 0;
        switch (key){
            case 'h': //home
                ctrl_flag_ = 1;
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "Move to HOME POSITION" << endl;
                cout << " " << endl;
                break;
            case 'a': //home
                ctrl_flag_ = 2;
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "Velocity Test" << endl;
                cout << " " << endl;
                break;
            
            case 'q': //quit
                ctrl_flag_ = 999;
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "Move to SAFT POSITION" << endl;
                cout << " " << endl;
                break;  
        }
    }
}
