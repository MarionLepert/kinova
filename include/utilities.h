// References: https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_cpp/examples/utilities.h

#ifndef KINOVA_UTILITIES_H
#define KINOVA_UTILITIES_H

#include <cxxopts.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <chrono>

int64_t GetTickUs();

Eigen::VectorXd to_pin_conf(const Eigen::Matrix<double,7,1>& q_rad);

struct ExampleArgs
{
    int robot_number;
    std::string config_path;
    std::string username;
    std::string password;
    // std::string redis_host;
};

ExampleArgs ParseExampleArguments(int argc, char *argv[]);



// Convert vector of numbers to space-separated string
std::string float_to_string(float value);

template<typename T>
std::string vector_to_string(const std::vector<T>& vec);

std::string eigen_vector_to_string(const Eigen::VectorXd& vec);

std::string eigen_quaternion_to_string(const Eigen::Quaterniond& quat);



Eigen::VectorXd string_to_eigen_vector(const std::string& str);

Eigen::Vector4d string_to_eigen_quaternion(const std::string& str);

template<typename T>
std::vector<T> string_to_vector(const std::string& str);

float string_to_float(const std::string& str);


// Print vector contents with optional prefix
template<typename T>
void print_vector(const std::vector<T>& vec, const std::string& prefix = "");

#endif //KINOVA_UTILITIES_H