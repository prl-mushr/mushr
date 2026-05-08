#ifndef ACKERMANN_CMD_MUX_SUBSCRIBERS_HPP_
#define ACKERMANN_CMD_MUX_SUBSCRIBERS_HPP_

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "ackermann_cmd_mux/exceptions.hpp"

namespace ackermann_cmd_mux {

class AckermannCmdSubscribers {
public:

    class AckermannCmdSubs {
    public:
        std::string name;
        std::string topic;
        double timeout;
        int priority;
        std::string short_desc;
        size_t index;

        AckermannCmdSubs(size_t idx)
        : index(idx)
        {}

        void operator<<(const YAML::Node & node) {
          name       = node["name"].as<std::string>();
          topic      = node["topic"].as<std::string>();
          timeout    = node["timeout"].as<double>();
          priority   = node["priority"].as<int>();
      
          if (node["short_desc"]) {
              short_desc = node["short_desc"].as<std::string>();
          }
      }
      
    };

    std::vector<AckermannCmdSubs> list;

    void configure(const YAML::Node & node);
};

} // namespace ackermann_cmd_mux

#endif
