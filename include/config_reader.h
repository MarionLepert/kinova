#pragma once

#include <string>
#include <map>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct RobotConfig {
    std::string ip_address;
    std::string redis_host;
};

struct RedisConfig {
    int port;
    std::string password;
};

struct Config {
    std::map<int, RobotConfig> robots;
    RedisConfig redis;
};

class ConfigReader {
public:
    static Config loadConfig(const std::string& config_path = "config/robot_config.json") {
        Config config;
        
        try {
            std::ifstream file(config_path);
            if (!file.is_open()) {
                throw std::runtime_error("Could not open config file: " + config_path);
            }
            
            json j;
            file >> j;
            
            // Load robot configurations
            for (const auto& [robot_id_str, robot_data] : j["robots"].items()) {
                int robot_id = std::stoi(robot_id_str);
                RobotConfig robot_config;
                robot_config.ip_address = robot_data["ip_address"];
                robot_config.redis_host = robot_data["redis_host"];
                config.robots[robot_id] = robot_config;
            }
            
            // Load Redis configuration
            config.redis.port = j["redis"]["port"];
            config.redis.password = j["redis"]["password"];
            
        } catch (const std::exception& e) {
            std::cerr << "Error loading config: " << e.what() << std::endl;
            throw;
        }
        
        return config;
    }
    
    static RobotConfig getRobotConfig(const Config& config, int robot_number) {
        auto it = config.robots.find(robot_number);
        if (it == config.robots.end()) {
            throw std::runtime_error("Robot configuration not found for robot number: " + std::to_string(robot_number));
        }
        return it->second;
    }
}; 