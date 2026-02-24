#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>

#include "distributed_satellite_sim/srv/actuation_cmd.hpp"

using ActuationCmd = distributed_satellite_sim::srv::ActuationCmd;

class EnvNode : public rclcpp::Node
{
public:
  EnvNode() : Node("env_node")
  {
    declare_parameter("max_steps", 91);
    max_steps_ = get_parameter("max_steps").as_int();

    init_dynamics();

    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("env_data", 10);

    cmd_srv_ = create_service<ActuationCmd>(
      "actuation_cmd",
      [this](const std::shared_ptr<ActuationCmd::Request> req,
             std::shared_ptr<ActuationCmd::Response> res) {
        actuation_callback(req, res);
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&EnvNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Env node started (max_steps=%d)", max_steps_);
  }

private:
  void init_dynamics()
  {
    x_now_ << 20, 20, 20, 0.00930458, -0.0467472, 0.00798343;
    u_now_ = Eigen::Vector3d::Zero();  // zero until first actuation_cmd call

    Ad_ <<
      1.25645279151274,      0, 0, 349.682205848334,  147.7805051,       0,
     -0.0716204647063059,    1, 0, -147.7805051,      318.728823393334,  0,
      0,                     0, 0.914515736162421, 0,  0,                349.682205848334,
      0.0014040831838806,    0, 0, 0.914515736162421,  0.809100657054006, 0,
     -0.00059338484671504,   0, 0, -0.809100657054006, 0.658062944649682, 0,
      0,                     0, -0.000468027727960198, 0, 0,              0.914515736162421;

    Bd_ <<
      63868.7072544296,  17836.8364281426, 0,
     -17836.8364281426,  61074.8290177183, 0,
      0,                 0,                63868.7072544296,
      349.682205848334,  147.7805051,      0,
     -147.7805051,       318.728823393334, 0,
      0,                 0,                349.682205848334;
  }

  void timer_callback()
  {
    if (max_steps_ > 0 && step_ >= max_steps_) {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "Simulation complete after %d steps", step_);
      return;
    }

    x_now_ = Ad_ * x_now_ + Bd_ * u_now_;

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.assign(x_now_.data(), x_now_.data() + x_now_.size());
    state_pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
      "step %d: x = [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
      step_,
      x_now_(0), x_now_(1), x_now_(2),
      x_now_(3), x_now_(4), x_now_(5));

    ++step_;
  }

  void actuation_callback(
    const std::shared_ptr<ActuationCmd::Request> req,
    std::shared_ptr<ActuationCmd::Response> res)
  {
    u_now_ << req->thrust[0], req->thrust[1], req->thrust[2];  // applied on next timer tick
    res->success = true;

    RCLCPP_INFO(get_logger(),
      "actuation_cmd received: u = [%.6e, %.6e, %.6e]",
      u_now_(0), u_now_(1), u_now_(2));
  }

  // Dynamics state
  Eigen::Matrix<double, 6, 1> x_now_;
  Eigen::Vector3d u_now_;
  int step_ = 0;
  int max_steps_;

  // Discrete dynamics matrices
  Eigen::Matrix<double, 6, 6> Ad_;
  Eigen::Matrix<double, 6, 3> Bd_;

  // ROS 2 interfaces
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Service<ActuationCmd>::SharedPtr cmd_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnvNode>());
  rclcpp::shutdown();
  return 0;
}
