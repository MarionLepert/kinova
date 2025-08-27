#ifndef REDIS_CLIENT_H
#define REDIS_CLIENT_H

#include <hiredis/hiredis.h>
#include <string>
#include <stdexcept>

class RedisClient {
public:
    RedisClient() : context_(nullptr) {}
    
    ~RedisClient() {
        if (context_) {
            redisFree(context_);
        }
    }

    void connect(const std::string& host, int port, const std::string& password = "") {
        context_ = redisConnect(host.c_str(), port);
        if (context_ == nullptr || context_->err) {
            if (context_) {
                std::string err = context_->errstr;
                redisFree(context_);
                context_ = nullptr;
                throw std::runtime_error("Failed to connect to Redis: " + err);
            }
            throw std::runtime_error("Failed to connect to Redis: Can't allocate redis context");
        }

        // Authenticate if password is provided
        if (!password.empty()) {
            redisReply* reply = (redisReply*)redisCommand(context_, "AUTH %s", password.c_str());
            if (reply == nullptr) {
                redisFree(context_);
                context_ = nullptr;
                throw std::runtime_error("Failed to authenticate with Redis");
            }
            if (reply->type == REDIS_REPLY_ERROR) {
                std::string err = reply->str;
                freeReplyObject(reply);
                redisFree(context_);
                context_ = nullptr;
                throw std::runtime_error("Redis authentication failed: " + err);
            }
            freeReplyObject(reply);
        }
    }

    void set(const std::string& key, const std::string& value) {
        if (!context_) {
            throw std::runtime_error("Not connected to Redis");
        }
        
        redisReply* reply = (redisReply*)redisCommand(context_, "SET %s %s", key.c_str(), value.c_str());
        if (reply == nullptr) {
            throw std::runtime_error("Failed to execute Redis command");
        }
        
        freeReplyObject(reply);
    }

    std::string get(const std::string& key) {
        if (!context_) {
            throw std::runtime_error("Not connected to Redis");
        }
        
        redisReply* reply = (redisReply*)redisCommand(context_, "GET %s", key.c_str());
        if (reply == nullptr) {
            throw std::runtime_error("Failed to execute Redis command");
        }
        
        std::string result;
        if (reply->type == REDIS_REPLY_STRING) {
            result = reply->str;
        }
        
        freeReplyObject(reply);
        return result;
    }

private:
    redisContext* context_;
};

#endif // REDIS_CLIENT_H 