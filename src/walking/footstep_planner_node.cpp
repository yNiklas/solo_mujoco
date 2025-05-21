// Plans when which foot should be placed where (without trajectory).
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "solo_mujoco/msg/footstep.hpp"  // auto-generated
#include "solo_mujoco/msg/footstep_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "builtin_interfaces/msg/time.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class FootstepPlannerNode : public rclcpp::Node
{
public:
  FootstepPlannerNode()
  : Node("footstep_planner")
  {
    pub_ = this->create_publisher<solo_mujoco::msg::FootstepArray>("planned_footsteps", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&FootstepPlannerNode::plan_steps, this));

    this->declare_parameter<double>("step_duration", 0.6);
    this->declare_parameter<double>("step_length", 0.1);
    this->declare_parameter<double>("default_z", 0.0);
  }

private:
  void plan_steps()
  {
    auto msg = solo_mujoco::msg::FootstepArray();
    auto now = this->get_clock()->now();

    double step_duration = this->get_parameter("step_duration").as_double();
    double step_length = this->get_parameter("step_length").as_double();
    double default_z = this->get_parameter("default_z").as_double();

    // Leg positions with respect to the base coordinate system.
    // Given by the URDF files.
    std::map<std::string, double> x_offsets = {
        {"FL",  0.1946}, {"FR", 0.1946},
        {"HL",  -0.1946}, {"HR", -0.1946}
    };
    std::map<std::string, double> y_offsets = {
        {"FL",  0.1015}, {"FR", -0.1015},
        {"HL",  0.1015}, {"HR", -0.1015}
    };

    for (int i = 0; i < 4; ++i) {
      std::vector<std::string> swing_legs = (i % 2 == 0) ? std::vector<std::string>{"FL", "HR"} : std::vector<std::string>{"FR", "HL"};

      rclcpp::Time t_start = now + rclcpp::Duration::from_seconds(i * step_duration);
      rclcpp::Time t_end = t_start + rclcpp::Duration::from_seconds(step_duration);

      for (const auto &leg : swing_legs) {
        solo_mujoco::msg::Footstep step;
        step.leg_name = leg;
        step.start_time = t_start;
        step.end_time = t_end;

        // Position in the base coordinate system (= in the middle of the robot)
        step.pose.position.x = (i + 1) * step_length + x_offsets[leg];
        step.pose.position.y = y_offsets[leg];
        step.pose.position.z = default_z; // z is calculated in the trajectory planner (not this node)
        step.pose.orientation.w = 1.0; // No orientation => identity quaternion

        msg.steps.push_back(step);
      }
    }

    pub_->publish(msg);
  }

  rclcpp::Publisher<solo_mujoco::msg::FootstepArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootstepPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
