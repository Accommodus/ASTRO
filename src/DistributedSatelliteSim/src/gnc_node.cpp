#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>

#include "distributed_satellite_sim/srv/actuation_cmd.hpp"

using ActuationCmd = distributed_satellite_sim::srv::ActuationCmd;

class GncNode : public rclcpp::Node
{
public:
  GncNode()
  : rclcpp::Node("gnc_node")
  {
    init_gain();

    state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "env_data",
      10,
      std::bind(&GncNode::state_callback, this, std::placeholders::_1));

    actuation_client_ = create_client<ActuationCmd>("actuation_cmd");

    RCLCPP_INFO(get_logger(), "GNC node started.");
  }

private:
  void init_gain()
  {
    // 3x6 control gain matrix K, copied from reference/DLQR/udp_roundtrip_discrete.cpp
    K_ << 
      4.66705488313515e-06,  -1.36363629843838e-06,  3.96098209388736e-22,
      0.00216637769262191,    0.000840447323685129,  7.35693727993496e-20,
      2.77991721268819e-06,   7.33444155253651e-07,  6.05699839509399e-22,
      0.000150571571893266,   0.00170682586161904,   1.18873969348727e-18,
     -3.71173845538396e-22,  -1.12623951370299e-21,  6.24116221637017e-07,
      1.62911699574378e-18,   9.12299177441715e-19,  0.00135188823063242;
  }

  void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 6) {
      RCLCPP_WARN(
        get_logger(),
        "Received env_data with size %zu (expected 6). Ignoring.",
        msg->data.size());
      return;
    }

    Eigen::Matrix<double, 6, 1> x;
    for (size_t i = 0; i < 6; ++i) {
      x(static_cast<Eigen::Index>(i)) = msg->data[i];
    }

    Eigen::Vector3d u = -K_ * x;

    RCLCPP_INFO(
      get_logger(),
      "env_data received: x = [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f] -> "
      "u = [%.6e, %.6e, %.6e]",
      x(0), x(1), x(2), x(3), x(4), x(5),
      u(0), u(1), u(2));

    if (!actuation_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "actuation_cmd service not available yet. Skipping control command.");
      return;
    }

    auto request = std::make_shared<ActuationCmd::Request>();
    request->thrust[0] = u(0);
    request->thrust[1] = u(1);
    request->thrust[2] = u(2);

    auto future = actuation_client_->async_send_request(
      request,
      [this, u](rclcpp::Client<ActuationCmd>::SharedFuture response_future) {
        auto response = response_future.get();
        if (response->success) {
          RCLCPP_INFO(
            this->get_logger(),
            "actuation_cmd service call succeeded for u = [%.6e, %.6e, %.6e]",
            u(0), u(1), u(2));
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "actuation_cmd service call reported failure for u = [%.6e, %.6e, %.6e]",
            u(0), u(1), u(2));
        }
      });

    (void)future;
  }

  Eigen::Matrix<double, 3, 6> K_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
  rclcpp::Client<ActuationCmd>::SharedPtr actuation_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GncNode>());
  rclcpp::shutdown();
  return 0;
}

