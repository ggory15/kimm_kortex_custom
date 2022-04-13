#include "example/move_to_home_position.hpp"

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

    n_node.getParam("/kimm_kortex_custum_home/port_number", PORT);
    n_node.getParam("/kimm_kortex_custum_home/ip_address", ip_address);
    n_node.getParam("/kimm_kortex_custum_home/username", username);
    n_node.getParam("/kimm_kortex_custum_home/password", password);
    test_string_pub_ = n_node.advertise<std_msgs::String>("my_kinova/test_string", 5);

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
                if (action.name() == "Offposition") 
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
