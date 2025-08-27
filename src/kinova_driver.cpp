// References: https://github.com/jimmyyhwu/tidybot2/blob/main/arm_controller.py

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm> 
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <Eigen/Dense>
#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <google/protobuf/util/json_util.h>
#include <ruckig/ruckig.hpp>
#include <ControlConfigClientRpc.h>

#include "include/utilities.h"
#include "include/redis_client.h"
#include "include/redis_keys.hpp"
#include "include/config_reader.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>

using namespace std;

namespace k_api = Kinova::Api;

#define MINIMAL_POSITION_ERROR  ((float)1.5)
#define PORT 10000
#define PORT_REAL_TIME 10001
#define DURATION 5             // Network timeout (seconds)

constexpr float kStepDeg   = 0.12f;
const auto ACTION_WAITING_TIME = std::chrono::seconds(1); // Waiting time during high-level actions
constexpr double PI_DEG  = 180.0; 

// Global variables
RedisClient* g_redis_client = nullptr;
int g_robot_number;
bool running = true;
std::atomic<bool> shutdown_requested{false};

// Create closure to set finished to true after an END or an ABORT
std::function<void(k_api::Base::ActionNotification)> 
check_for_end_or_abort(bool& finished)
{
    return [&finished](k_api::Base::ActionNotification notification)
    {
        std::cout << "EVENT : " << k_api::Base::ActionEvent_Name(notification.action_event()) << std::endl;

        // The action is finished when we receive a END or ABORT event
        switch(notification.action_event())
        {
        case k_api::Base::ActionEvent::ACTION_ABORT:
        case k_api::Base::ActionEvent::ACTION_END:
            finished = true;
            break;
        default:
            break;
        }
    };
}

void move_home(k_api::Base::BaseClient* base)
// Move the arm to the default home position provided by the Kinova API
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    } 
    else 
    {
        bool action_finished = false; 
        // Notify of any action topic event
        auto options = k_api::Common::NotificationOptions();
        auto notification_handle = base->OnNotificationActionTopic(
            check_for_end_or_abort(action_finished),
            options
        );

        base->ExecuteActionFromReference(action_handle);

        while(!action_finished)
        { 
            std::this_thread::sleep_for(ACTION_WAITING_TIME);
        }

        base->Unsubscribe(notification_handle);
    }
}

struct Proprioception {
    std::vector<float> joint_angles;
    float gripper_position;
};

Proprioception get_proprioception(k_api::Base::BaseClient* base,  k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    k_api::BaseCyclic::Feedback base_feedback;
    base_feedback = base_cyclic->RefreshFeedback();
    std::vector<float> joint_angles;
    for (int i = 0; i < base_feedback.actuators_size(); i++)
    {
        joint_angles.push_back(base_feedback.actuators(i).position());
    }

    float gripper_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();

    return Proprioception{joint_angles, gripper_position};
}

struct EEPose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

EEPose get_ee_pose(pinocchio::Model& model, pinocchio::Data& data, int tool_id, std::vector<float>& q_s)
{
    // Convert std vector to pinocchio vector
    Eigen::Matrix<double,7,1> q_rad;
    for(int i = 0; i < 7; i++)
    {
        q_rad[i] = q_s[i] * M_PI / 180.0;
    }
    Eigen::VectorXd q_pin = to_pin_conf(q_rad);

    // Forward kinematics
    pinocchio::framesForwardKinematics(model, data, to_pin_conf(q_rad));
    const pinocchio::SE3& T = data.oMf[tool_id];

    // Extract pose
    EEPose pose;
    pose.position = T.translation();
    pose.orientation = Eigen::Quaterniond(T.rotation());   // (w,x,y,z)

    return pose;
}

double unwrap_angle(double current_angle, double commanded_angle) {
    // Calculate the shortest angular difference
    double diff = commanded_angle - current_angle;
    
    // Normalize difference to [-180, 180] to find shortest path
    while (diff > PI_DEG) {
        diff -= 2.0 * PI_DEG;
    }
    while (diff < -PI_DEG) {
        diff += 2.0 * PI_DEG;
    }
    
    // Add the difference to current angle to get unwrapped result
    return current_angle + diff;
}

void move_home_bimanual_angled(k_api::Base::BaseClient* base,  k_api::BaseCyclic::BaseCyclicClient* base_cyclic, RedisClient& redis_client, int robot_number, int robot_number_2)
// Move the arm to the home position used for our bimanual setup
{
    const std::string& REDIS_KEY_STATUS = (robot_number == 1) ? REDIS_KEY_BOT1_STATUS : REDIS_KEY_BOT2_STATUS;

    auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback &data)
    {
        return;
    };

    int timeout = 0;
    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;
    int64_t start = 0;

    int actuator_count = base->GetActuatorCount().count();

    k_api::BaseCyclic::Command base_command;
    Proprioception proprioception = get_proprioception(base, base_cyclic);

    // Initialize joint commands to their current positions
    std::vector<float> q_s = proprioception.joint_angles;
    std::vector<float> commands;
    for(int i = 0; i < actuator_count; i++)
    {
        commands.push_back(q_s[i]);
        base_command.add_actuators()->set_position(q_s[i]);
    }

    // Initialize gripper command to its current position
    base_command.mutable_interconnect()->mutable_command_id()->set_identifier(0);
    auto gripper_motor_command = base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
    gripper_motor_command->set_position(proprioception.gripper_position);
    gripper_motor_command->set_velocity(0.0);
    gripper_motor_command->set_force(100.0);
    float gripper_position_command = proprioception.gripper_position;

    // Initialize Ruckig
    const double DT = 0.001; // 1ms control period
    ruckig::Ruckig<7> otg(DT);
    ruckig::InputParameter<7> otg_inp;
    ruckig::OutputParameter<7> otg_out;

    // Set velocity and acceleration limits
    std::array<double, 7> max_velocity = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}; // degrees/s
    std::array<double, 7> max_acceleration = {60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0}; // degrees/s^2
    
    for (int i = 0; i < 7; i++) {
        otg_inp.max_velocity[i] = max_velocity[i];
        otg_inp.max_acceleration[i] = max_acceleration[i];
        otg_inp.current_position[i] = q_s[i];
        otg_inp.current_velocity[i] = 0.0;
        otg_inp.target_position[i] = q_s[i];
        otg_inp.target_velocity[i] = 0.0;
    }

    ruckig::Result otg_res = ruckig::Result::Working;

    // Real-time loop
    const int FREQ_WINDOW = 1000;  // Number of iterations to average over
    std::vector<double> loop_times;
    loop_times.reserve(FREQ_WINDOW);

    std::vector<float> q_command;
    if (robot_number == 1) {
        q_command = {121.0, 52.0, 190.0, 246.0, 302.0, 298.0, 181.0};
    } else {
        q_command = {236.0, 51.0, 175.0, 250.0, 50.0, 301.0, 16.0};
    }

    start = GetTickUs();
    now = GetTickUs();
    while(now - start < 20000000)
    {
        now = GetTickUs();
        if(now - last >= 1000) // Run at 1000 Hz (1ms period)
        {
            // Calculate loop time and frequency
            double loop_time = (now - last) / 1000.0;  // Convert to milliseconds
            loop_times.push_back(loop_time);
            if (loop_times.size() > FREQ_WINDOW) {
                loop_times.erase(loop_times.begin());
            }

            // Get current positions
            Proprioception proprioception = get_proprioception(base, base_cyclic);
            q_s = proprioception.joint_angles;
            float gripper_position = proprioception.gripper_position;

            // Update Ruckig target if we have a new command
            for (int i = 0; i < actuator_count; i++) {
                double unwrapped_q = unwrap_angle(otg_inp.current_position[i], q_command[i]);
                otg_inp.target_position[i] = unwrapped_q;
            }
            otg_res = ruckig::Result::Working;

            // Update Ruckig trajectory
            if (otg_res == ruckig::Result::Working) {
                otg_res = otg.update(otg_inp, otg_out);
                otg_out.pass_to_input(otg_inp);

                // Set joint commands from Ruckig output
                for (int i = 0; i < actuator_count; i++) {
                    float new_position = otg_out.new_position[i];
                    base_command.mutable_actuators(i)->set_position(new_position);
                }
            }

            // Set gripper command
            float velocity;
            float proportional_gain = 2.0;

            float gripper_position_error = gripper_position_command - gripper_position;

            velocity = proportional_gain * fabs(gripper_position_error);
            if (velocity > 100.0)
            {
                velocity = 100.0;
            }
            
            gripper_motor_command->set_position(gripper_position_command);
            gripper_motor_command->set_velocity(velocity);

            try
            {
                base_cyclic->Refresh_callback(base_command, lambda_fct_callback, 0);
            }
            catch(...)
            {
                timeout++;
            }

            // Check if the status is 0
            std::string status_str = redis_client.get(REDIS_KEY_STATUS);
            if (status_str == "0") {
                std::cout << "Status is 0, stopping control loop" << std::endl;
                break;
            }

            // Check if all joint errors are below threshold
            bool all_joints_within_threshold = true;
            for (int i = 0; i < actuator_count; i++) {
                if (fabs(q_command[i] - q_s[i]) > MINIMAL_POSITION_ERROR) {
                    all_joints_within_threshold = false;
                    break;
                }
            }
            if (all_joints_within_threshold) {
                break;
            }

            timer_count++;
            last = now;
        }
    }
}

// Shared data structure
struct SharedDataSensing {
    std::mutex mutex;
    std::vector<float> q_s;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    float gripper_position;
};

struct SharedDataCommand {
    std::mutex mutex;
    std::vector<float> q_command; 
    float gripper_position_command;
    std::string status_str;
};

void redis_thread_command(RedisClient& redis_client, SharedDataCommand& shared_data, 
                         const std::string& REDIS_KEY_Q_DES, const std::string& REDIS_KEY_GRIPPER_POS_DES, 
                         const std::string& REDIS_KEY_STATUS) {
    while(running && !shutdown_requested.load()) {
        // SLOW: Redis operations (no lock held)
        std::string q_command_str = redis_client.get(REDIS_KEY_Q_DES);
        std::string gripper_position_command_str = redis_client.get(REDIS_KEY_GRIPPER_POS_DES);
        std::string status_str = redis_client.get(REDIS_KEY_STATUS);

        // Validate data before parsing
        if (!q_command_str.empty() && !gripper_position_command_str.empty() && !status_str.empty()) {
            std::vector<float> q_command = string_to_vector<float>(q_command_str);
            float gripper_position_command = string_to_float(gripper_position_command_str);

            // FAST: Update shared memory (minimal lock time)
            {
                std::lock_guard<std::mutex> lock(shared_data.mutex);
                shared_data.q_command = q_command;
                shared_data.gripper_position_command = gripper_position_command * 100.0;
                shared_data.status_str = status_str;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void redis_thread_sensing(RedisClient& redis_client, SharedDataSensing& shared_data, 
                          const std::string& REDIS_KEY_Q, const std::string& REDIS_KEY_EE_POS, 
                          const std::string& REDIS_KEY_EE_QUAT_WXYZ, const std::string& REDIS_KEY_GRIPPER_POS, 
                          const std::string& REDIS_KEY_STATUS) {
    while(running && !shutdown_requested.load()) {
        // Local copies to minimize lock time
        std::vector<float> local_q_s;
        Eigen::Vector3d local_position;
        Eigen::Quaterniond local_orientation;
        float local_gripper_position;
        bool data_available = false;

        // FAST: Copy data from shared memory (minimal lock time)
        {
            std::lock_guard<std::mutex> lock(shared_data.mutex);
            local_q_s = shared_data.q_s;
            local_position = shared_data.position;
            local_orientation = shared_data.orientation;
            local_gripper_position = shared_data.gripper_position / 100.0;
            data_available = true;
        }

        // SLOW: Redis operations (no lock held)
        if (data_available) {
            std::string q_s_str = vector_to_string(local_q_s);
            std::string ee_pos_str = eigen_vector_to_string(local_position);
            std::string ee_ori_str = eigen_quaternion_to_string(local_orientation);
            std::string gripper_pos_str = std::to_string(local_gripper_position);

            redis_client.set(REDIS_KEY_Q, q_s_str.c_str());
            redis_client.set(REDIS_KEY_EE_POS, ee_pos_str.c_str());
            redis_client.set(REDIS_KEY_EE_QUAT_WXYZ, ee_ori_str.c_str());
            redis_client.set(REDIS_KEY_GRIPPER_POS, gripper_pos_str.c_str());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

class HighResTimer {
private:
    std::chrono::high_resolution_clock::time_point start_time;
    
public:
    HighResTimer() {
        start_time = std::chrono::high_resolution_clock::now();
    }
    
    // Returns microseconds since timer creation
    int64_t GetMicroseconds() {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time);
        return duration.count();
    }
    
    // Reset the timer
    void Reset() {
        start_time = std::chrono::high_resolution_clock::now();
    }
};


void run(k_api::Base::BaseClient* base,  k_api::BaseCyclic::BaseCyclicClient* base_cyclic, 
        RedisClient& redis_client, int robot_number, pinocchio::Model& model, pinocchio::Data& data, int tool_id, 
        SharedDataSensing& shared_data_sensing, SharedDataCommand& shared_data_command, const std::string& REDIS_KEY_Q_DES)
{
    auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback &data)
    {
        return;
    };

    int actuator_count = base->GetActuatorCount().count();

    k_api::BaseCyclic::Command base_command;
    Proprioception proprioception = get_proprioception(base, base_cyclic);

    base_command.mutable_interconnect()->mutable_command_id()->set_identifier(0);

    // Initialize joint commands to their current positions
    std::vector<float> q_s = proprioception.joint_angles;
    std::vector<float> commands;
    for(int i = 0; i < actuator_count; i++)
    {
        commands.push_back(q_s[i]);
        base_command.add_actuators()->set_position(q_s[i]);
    }
    float gripper_position = proprioception.gripper_position;

    // Initialize gripper command to its current position
    base_command.mutable_interconnect()->mutable_command_id()->set_identifier(0);
    auto gripper_motor_command = base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
    gripper_motor_command->set_position(proprioception.gripper_position);
    gripper_motor_command->set_velocity(0.0);
    gripper_motor_command->set_force(100.0);

    // Initialize redis command to current joint angles
    std::string q_s_str = vector_to_string(q_s);
    redis_client.set(REDIS_KEY_Q_DES, q_s_str);

    // Initialize Ruckig
    const double DT = 0.001; // 1ms control period
    ruckig::Ruckig<7> otg(DT);
    ruckig::InputParameter<7> otg_inp;
    ruckig::OutputParameter<7> otg_out;

    // Set velocity and acceleration limits
    std::array<double, 7> max_velocity = {250.0, 250.0, 250.0, 250.0, 400.0, 400.0, 400.0}; // degrees/s
    std::array<double, 7> max_acceleration = {1500.0, 1500.0, 1500.0, 2000.0, 2500.0, 2500.0, 2500.0}; // degrees/s^2
    
    for (int i = 0; i < 7; i++) {
        otg_inp.max_velocity[i] = max_velocity[i];
        otg_inp.max_acceleration[i] = max_acceleration[i];
        otg_inp.current_position[i] = q_s[i];
        otg_inp.current_velocity[i] = 0.0;
        otg_inp.target_position[i] = q_s[i];
        otg_inp.target_velocity[i] = 0.0;
    }

    ruckig::Result otg_res = ruckig::Result::Working;

    float gripper_position_command;
    std::vector<float> q_command;
    std::string status_str;


    HighResTimer timer;
    const int64_t TARGET_PERIOD_US = 1000; // 1ms = 1000Hz
    int64_t loop_counter = 0;
    int64_t next_target_time = TARGET_PERIOD_US;

    // Statistics tracking
    std::vector<double> timing_errors;
    timing_errors.reserve(1000);
    int64_t max_timing_error = 0;
    int64_t total_timing_error = 0;

    while(running && !shutdown_requested.load())
    {
        int64_t now = timer.GetMicroseconds();

        if(now >= next_target_time) // Run at 1000 Hz (1ms period)
        {
            // Calculate timing error for statistics
            int64_t timing_error = now - next_target_time;
            if (timing_error > max_timing_error) max_timing_error = timing_error;
            total_timing_error += abs(timing_error);
            
            timing_errors.push_back(timing_error);
            if (timing_errors.size() > 1000) {
                timing_errors.erase(timing_errors.begin());
            }

            // Calculate next target time BEFORE doing any work
            loop_counter++;
            next_target_time = loop_counter * TARGET_PERIOD_US;

            // Print timing statistics every 5 seconds (5000 iterations at 1kHz)
            if (loop_counter % 5000 == 0) {
                double total_elapsed_time_seconds = double(now) / 1000000.0;
                double actual_frequency = double(loop_counter) / total_elapsed_time_seconds;
                
                double avg_timing_error = timing_errors.empty() ? 0.0 : 
                    std::accumulate(timing_errors.begin(), timing_errors.end(), 0.0) / timing_errors.size();
                
                std::cout << "[" << std::fixed << std::setprecision(1) << total_elapsed_time_seconds << "s] "
                         << "Frequency: " << std::setprecision(6) << actual_frequency << " Hz "
                         << "(target: 1000.000 Hz) | "
                         << "Avg timing error: " << std::setprecision(1) << avg_timing_error << " μs | "
                         << "Max error: " << max_timing_error << " μs" << std::endl;
                         
                max_timing_error = 0; // Reset max error counter for next interval
            }

            // Get redis command 
            {
                std::lock_guard<std::mutex> lock(shared_data_command.mutex);
                q_command = shared_data_command.q_command;
                gripper_position_command = shared_data_command.gripper_position_command;
                status_str = shared_data_command.status_str;
            }

            if (q_command.size() != 7) {
                std::cerr << "Error: q_command size is " << q_command.size() << ", expected 7. Skipping iteration." << std::endl;
                running = false;
                break;
            }

            // Check if we should stop
            if (status_str == "0" || shutdown_requested.load()) {
                std::cout << "Status is 0, stopping control loop" << std::endl;
                break;
            }

            // Update Ruckig target if we have a new command
            for (int i = 0; i < actuator_count; i++) {
                otg_inp.current_position[i] = q_s[i];
                double unwrapped_q = unwrap_angle(q_s[i], q_command[i]);
                otg_inp.target_position[i] = unwrapped_q;
            }
            otg_res = ruckig::Result::Working;

            // Increment frame ID to ensure actuators can reject out-of-order frames
            base_command.mutable_interconnect()->mutable_command_id()->set_identifier((base_command.mutable_interconnect()->mutable_command_id()->identifier() + 1) % 65536);

            // Update Ruckig trajectory
            if (otg_res == ruckig::Result::Working) {
                otg_res = otg.update(otg_inp, otg_out);
                otg_out.pass_to_input(otg_inp);

                // Set joint commands from Ruckig output
                for (int i = 0; i < actuator_count; i++) {
                    float new_position = otg_out.new_position[i];
                    base_command.mutable_actuators(i)->set_position(new_position);
                }
            }

            // Set gripper command
            float velocity;
            float proportional_gain = 2.0;

            float gripper_position_error = gripper_position_command - gripper_position;
            velocity = proportional_gain * fabs(gripper_position_error);
            if (velocity > 100.0)
            {
                velocity = 100.0;
            }
            
            gripper_motor_command->set_position(gripper_position_command);
            gripper_motor_command->set_velocity(velocity);

            k_api::BaseCyclic::Feedback base_feedback = base_cyclic->Refresh(base_command);

            for (int i = 0; i < base_feedback.actuators_size(); i++)
            {
                q_s[i] = base_feedback.actuators(i).position();
            }

            gripper_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();

            // Update redis
            {
                std::lock_guard<std::mutex> lock(shared_data_sensing.mutex);
                shared_data_sensing.q_s = q_s;
                shared_data_sensing.position = get_ee_pose(model, data, tool_id, q_s).position;
                shared_data_sensing.orientation = get_ee_pose(model, data, tool_id, q_s).orientation;
                shared_data_sensing.gripper_position = gripper_position;
            }

        } else {
            // We're ahead of schedule - yield CPU briefly
            // This prevents busy-waiting and reduces CPU usage
            std::this_thread::sleep_for(std::chrono::microseconds(2));
        }
    }
}



struct KinovaRobot {
    k_api::Base::BaseClient* base;
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic;
    k_api::SessionManager* session_manager;
    k_api::SessionManager* session_manager_real_time;
    k_api::RouterClient* router;
    k_api::RouterClient* router_real_time;
    k_api::TransportClientTcp* transport;
    k_api::TransportClientUdp* transport_real_time;
};

std::unique_ptr<KinovaRobot> g_robot;


void signal_handler(int signum) {
    if (signum == SIGINT) {
        std::cout << "\nReceived CTRL-C. Cleaning up..." << std::endl;
        shutdown_requested.store(true);        
    }
}

KinovaRobot init_kinova(const std::string& ip_address, const std::string& username, const std::string& password) {
    // Create API objects
    auto error_callback = [](k_api::KError err){ std::cout << "_________ callback error _________" << err.toString(); };
    
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(ip_address, PORT);

    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(username);
    create_session_info.set_password(password);
    create_session_info.set_session_inactivity_timeout(600000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(20000); // (milliseconds)

    // Session manager service wrapper
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);

    auto control_config = new k_api::ControlConfig::ControlConfigClient(router);
    k_api::ControlConfig::GravityVector gravity_vector = control_config->GetGravityVector();

    // Display the gravity vector
    std::cout << "########################################################" << std::endl;
    std::cout << "Gravity Vector: [" << gravity_vector.x() << ", " << gravity_vector.y() << ", " << gravity_vector.z() << "]" << std::endl;

    return KinovaRobot{
        base,
        base_cyclic,
        session_manager,
        session_manager_real_time,
        router,
        router_real_time,
        transport,
        transport_real_time
    };
}

RedisClient init_redis(const std::string& host = "127.0.0.1", int port = 6379, const std::string& password = "iprl") {
    RedisClient redis_client = RedisClient();
    redis_client.connect(host, port, password);
    return redis_client;
}

int main(int argc, char **argv)
{
    // Set up signal handler
    signal(SIGINT, signal_handler);

    auto parsed_args = ParseExampleArguments(argc, argv);
    
    // Load configuration from file
    Config config;
    try {
        config = ConfigReader::loadConfig(parsed_args.config_path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load configuration from '" << parsed_args.config_path << "': " << e.what() << std::endl;
        return 1;
    }
    
    // Get robot configuration
    RobotConfig robot_config;
    try {
        robot_config = ConfigReader::getRobotConfig(config, parsed_args.robot_number);
    } catch (const std::exception& e) {
        std::cerr << "Failed to get robot configuration: " << e.what() << std::endl;
        return 1;
    }
    
    std::string ip_address = robot_config.ip_address;
    std::string redis_host = robot_config.redis_host;

    g_robot_number = parsed_args.robot_number;  // Store the robot number
    g_robot = std::make_unique<KinovaRobot>(init_kinova(ip_address, parsed_args.username, parsed_args.password));
    if (!g_robot) {
        std::cerr << "Failed to initialize Kinova robot" << std::endl;
        return 1;
    }

    // Get the appropriate Redis keys based on robot number
    const std::string& REDIS_KEY_Q = (g_robot_number == 1) ? REDIS_KEY_BOT1_Q : REDIS_KEY_BOT2_Q;
    const std::string& REDIS_KEY_Q_DES = (g_robot_number == 1) ? REDIS_KEY_BOT1_Q_DES : REDIS_KEY_BOT2_Q_DES;
    const std::string& REDIS_KEY_EE_POS = (g_robot_number == 1) ? REDIS_KEY_BOT1_EE_POS : REDIS_KEY_BOT2_EE_POS;
    const std::string& REDIS_KEY_EE_QUAT_WXYZ = (g_robot_number == 1) ? REDIS_KEY_BOT1_EE_QUAT_WXYZ : REDIS_KEY_BOT2_EE_QUAT_WXYZ;
    const std::string& REDIS_KEY_GRIPPER_POS = (g_robot_number == 1) ? REDIS_KEY_BOT1_GRIPPER_POS : REDIS_KEY_BOT2_GRIPPER_POS;
    const std::string& REDIS_KEY_GRIPPER_POS_DES = (g_robot_number == 1) ? REDIS_KEY_BOT1_GRIPPER_POS_DES : REDIS_KEY_BOT2_GRIPPER_POS_DES;
    const std::string& REDIS_KEY_STATUS = (g_robot_number == 1) ? REDIS_KEY_BOT1_STATUS : REDIS_KEY_BOT2_STATUS;

    // Initialize Redis client
    g_redis_client = new RedisClient(init_redis(redis_host, config.redis.port, config.redis.password));
    RedisClient redis_client_sensing = init_redis(redis_host, config.redis.port, config.redis.password);   // For sensing thread
    RedisClient redis_client_command = init_redis(redis_host, config.redis.port, config.redis.password);   // For command thread

    g_redis_client->set(REDIS_KEY_STATUS, "1");

    // Set the base in low-level servoing mode
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    g_robot->base->SetServoingMode(servoingMode);

    // Initialize the robot
    move_home_bimanual_angled(g_robot->base, g_robot->base_cyclic, *g_redis_client, g_robot_number, parsed_args.robot_number);

    Proprioception proprioception = get_proprioception(g_robot->base, g_robot->base_cyclic);
    std::vector<float>q_s = proprioception.joint_angles;
    std::string q_s_str = vector_to_string(q_s);
    std::cout << std::endl;
    std::cout << "Joint angles: " << q_s_str << std::endl;
    std::cout << "Gripper position: " << proprioception.gripper_position << std::endl;

    // Initialize pinocchio model
    pinocchio::Model model;
    pinocchio::urdf::buildModel("../models/gen3_robotiq_2f_85.urdf", model);
    pinocchio::Data data(model);
    const int tool_id = model.getFrameId("tool_frame");

    EEPose ee_pose = get_ee_pose(model, data, tool_id, q_s);
    std::cout << "EE pose: " << ee_pose.position.x() << ", " << ee_pose.position.y() << ", " << ee_pose.position.z() << std::endl;
    std::cout << "EE orientation: " << ee_pose.orientation.x() << ", " << ee_pose.orientation.y() << ", " << ee_pose.orientation.z() << ", " << ee_pose.orientation.w() << std::endl;
    std::cout << "########################################################" << std::endl;

    // Initialize redis keys
    g_redis_client->set(REDIS_KEY_Q, q_s_str.c_str());
    g_redis_client->set(REDIS_KEY_Q_DES, q_s_str.c_str());
    g_redis_client->set(REDIS_KEY_EE_POS, eigen_vector_to_string(ee_pose.position).c_str());
    g_redis_client->set(REDIS_KEY_EE_QUAT_WXYZ, eigen_quaternion_to_string(ee_pose.orientation).c_str());
    g_redis_client->set(REDIS_KEY_GRIPPER_POS, "0.0");
    g_redis_client->set(REDIS_KEY_GRIPPER_POS_DES, "0.0");

    // Initialize shared data
    SharedDataSensing shared_data_sensing;
    SharedDataCommand shared_data_command;

    // Initialize Redis threads
    std::thread sensing_thread(redis_thread_sensing, std::ref(redis_client_sensing), std::ref(shared_data_sensing), REDIS_KEY_Q, REDIS_KEY_EE_POS, REDIS_KEY_EE_QUAT_WXYZ, REDIS_KEY_GRIPPER_POS, REDIS_KEY_STATUS);
    std::thread command_thread(redis_thread_command, std::ref(redis_client_command), std::ref(shared_data_command), REDIS_KEY_Q_DES, REDIS_KEY_GRIPPER_POS_DES, REDIS_KEY_STATUS);

    std::this_thread::sleep_for(std::chrono::seconds(1)); // todo: is this necessary?
    std::cout << "CONTROL LOOP STARTED" << std::endl;

    // Run the control loop
    run(g_robot->base, g_robot->base_cyclic, *g_redis_client, g_robot_number, model, data, tool_id, shared_data_sensing, shared_data_command, REDIS_KEY_Q_DES);

    running = false;

    // Set Redis status to 0 (in case signal handler couldn't do it safely)
    if (g_redis_client != nullptr) {
        const std::string& REDIS_KEY_STATUS = (g_robot_number == 1) ? REDIS_KEY_BOT1_STATUS : REDIS_KEY_BOT2_STATUS;
        try {
            g_redis_client->set(REDIS_KEY_STATUS, "0");
        } catch (...) {
            std::cerr << "Warning: Could not set Redis status to 0" << std::endl;
        }
    }

    // Wait for threads to finish (with timeout)
    auto join_with_timeout = [](std::thread& t, const std::string& name) {
        if (t.joinable()) {
            std::cout << "Waiting for " << name << " thread to finish..." << std::endl;
            
            // Try to join with timeout
            auto future = std::async(std::launch::async, [&t]() { t.join(); });
            
            if (future.wait_for(std::chrono::seconds(2)) == std::future_status::timeout) {
                std::cerr << "Warning: " << name << " thread did not finish within timeout" << std::endl;
                // Thread will be terminated when program exits
            } else {
                std::cout << name << " thread finished cleanly" << std::endl;
            }
        }
    };
    
    join_with_timeout(sensing_thread, "sensing");
    join_with_timeout(command_thread, "command");

    // Clean up robot resources
    if (g_robot) {
        try {
            g_robot->base->Stop();
            
            // Close API sessions
            g_robot->session_manager->CloseSession();
            g_robot->session_manager_real_time->CloseSession();

            // Deactivate routers and disconnect
            g_robot->router->SetActivationStatus(false);
            g_robot->transport->disconnect();
            g_robot->router_real_time->SetActivationStatus(false);
            g_robot->transport_real_time->disconnect();

            // Clean up API objects
            delete g_robot->base;
            delete g_robot->base_cyclic;
            delete g_robot->session_manager;
            delete g_robot->session_manager_real_time;
            delete g_robot->router;
            delete g_robot->router_real_time;
            delete g_robot->transport;
            delete g_robot->transport_real_time;
            
        } catch (const std::exception& e) {
            std::cerr << "Error during robot cleanup: " << e.what() << std::endl;
        }
    }
    
    // Clean up Redis client
    if (g_redis_client) {
        delete g_redis_client;
        g_redis_client = nullptr;
    }

    std::cout << "Cleanup complete." << std::endl;
    return shutdown_requested.load() ? 1 : 0;  // Return 1 if interrupted, 0 if normal exit
}
