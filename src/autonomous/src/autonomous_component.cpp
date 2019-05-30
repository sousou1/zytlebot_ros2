
#include "autonomous/autonomous_component.hpp"

#include <class_loader/register_macro.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace autonomous
{

    Autonomous::Autonomous()
            : Node("autonomous")
    {
        timer_ = create_wall_timer(1s, std::bind(&Autonomous::broadcast_greeting, this));
    }

    void Autonomous::broadcast_greeting()
    {
    }

} // namespace autonomous

CLASS_LOADER_REGISTER_CLASS(autonomous::Autonomous, rclcpp::Node)