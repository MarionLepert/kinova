#include <cstdlib>
#include "include/utilities.h"
#include <sstream>
#include <Eigen/Dense>

int64_t GetTickUs()
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

// Helper: map 7 revolute-joint angles ➜ Gen3 "cos-sin / angle" configuration
Eigen::VectorXd to_pin_conf(const Eigen::Matrix<double,7,1>& q_rad)
{
    Eigen::VectorXd q_pin(11);           // nq of Gen3 URDF
    q_pin << std::cos(q_rad[0]), std::sin(q_rad[0]),
             q_rad[1],
             std::cos(q_rad[2]), std::sin(q_rad[2]),
             q_rad[3],
             std::cos(q_rad[4]), std::sin(q_rad[4]),
             q_rad[5],
             std::cos(q_rad[6]), std::sin(q_rad[6]);
    return q_pin;
}

//Helper function to parse program arguments
ExampleArgs ParseExampleArguments(int argc, char *argv[])
{
    cxxopts::Options options(argv[0], "Kortex example");
    
    options.add_options()
        ("r,robot", "robot number", cxxopts::value<int>()->default_value("1"))
        // ("ip", "IP address of destination", cxxopts::value<std::string>()->default_value(""))
        ("c,config", "Path to configuration file", cxxopts::value<std::string>()->default_value("config/robot_config.json"))
        ("h,help", "Print help")
        ("u,username", "username to login", cxxopts::value<std::string>()->default_value("admin"))
        ("p,password", "password to login", cxxopts::value<std::string>()->default_value("admin"))
        // ("redis", "Redis host", cxxopts::value<std::string>()->default_value("laptop"))
    ;

    ExampleArgs resultArgs;

    try
    {
        auto parsed_options = options.parse(argc, argv);

        if (parsed_options.count("help"))
        {
            std::cout << options.help() << std::endl;
            exit(EXIT_SUCCESS);
        }

        resultArgs.robot_number = parsed_options["robot"].as<int>();
        // resultArgs.ip_address = parsed_options["ip"].as<std::string>();
        resultArgs.config_path = parsed_options["config"].as<std::string>();
        resultArgs.username = parsed_options["username"].as<std::string>();
        resultArgs.password = parsed_options["password"].as<std::string>();
        // resultArgs.redis_host = parsed_options["redis"].as<std::string>();
    }
    catch(const cxxopts::OptionException& exception)
    {
        std::cerr << exception.what() << std::endl;
        exit(EXIT_FAILURE);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        exit(EXIT_FAILURE);
    }
    
    return resultArgs;
}

std::string float_to_string(float value) {
    std::stringstream ss;
    ss << value;
    return ss.str();
}

template<typename T>
std::string vector_to_string(const std::vector<T>& vec) {
    std::stringstream ss;
    for (int i = 0; i < int(vec.size()); i++) {
        ss << vec[i];
        if (i < vec.size() - 1) {
            ss << " ";
        }
    }
    return ss.str();
}

std::string eigen_vector_to_string(const Eigen::VectorXd& vec) {
    std::stringstream ss;
    for (int i = 0; i < int(vec.size()); i++) {
        ss << vec[i];
        if (i < vec.size() - 1) {
            ss << " ";
        }
    }
    return ss.str();
}

std::string eigen_quaternion_to_string(const Eigen::Quaterniond& quat) {
    std::stringstream ss;
    ss << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z();
    return ss.str();
}

template<typename T>
std::vector<T> string_to_vector(const std::string& str) {
    std::vector<T> vec;
    std::stringstream ss(str);
    T value;
    while (ss >> value) {
        vec.push_back(value);
        if (ss.peek() == ' ') {
            ss.ignore();
        }
    }
    return vec;
}

float string_to_float(const std::string& str) {
    std::stringstream ss(str);
    float value;
    ss >> value;
    return value;
}

Eigen::VectorXd string_to_eigen_vector(const std::string& str) {
    std::vector<double> vec = string_to_vector<double>(str);
    return Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());
}

Eigen::Vector4d string_to_eigen_quaternion(const std::string& str) {
    std::vector<double> vec = string_to_vector<double>(str);
    return Eigen::Map<Eigen::Vector4d>(vec.data(), vec.size());
}

template<typename T>
void print_vector(const std::vector<T>& vec, const std::string& prefix) {
    for (size_t i = 0; i < vec.size(); i++) {
        std::cout << prefix << "[" << i << "]: " << vec[i] << std::endl;
    }
}

// Explicit template instantiations for common types
template std::string vector_to_string<float>(const std::vector<float>& vec);
template std::string vector_to_string<double>(const std::vector<double>& vec);
template std::string vector_to_string<int>(const std::vector<int>& vec);

template std::vector<float> string_to_vector<float>(const std::string& str);
template std::vector<double> string_to_vector<double>(const std::string& str);
template std::vector<int> string_to_vector<int>(const std::string& str);

template void print_vector<float>(const std::vector<float>& vec, const std::string& prefix);
template void print_vector<double>(const std::vector<double>& vec, const std::string& prefix);
template void print_vector<int>(const std::vector<int>& vec, const std::string& prefix);