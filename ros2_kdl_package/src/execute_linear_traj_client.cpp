#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "ros2_kdl_interfaces/action/execute_linear_traj.hpp"
#include <chrono>
#include <string>
#include <vector>

namespace ros2_kdl_pkg {

class ExecuteLinearTrajClient : public rclcpp::Node {
public:
  using ActionT     = ros2_kdl_interfaces::action::ExecuteLinearTraj;
  using GoalHandleT = rclcpp_action::ClientGoalHandle<ActionT>;
  GoalHandleT::SharedPtr goal_handle_;
  explicit ExecuteLinearTrajClient(const rclcpp::NodeOptions & options)
  : rclcpp::Node("execute_linear_traj_client", options)
  {
    action_name_ = this->declare_parameter<std::string>("action_name", "execute_linear_traj");

    client_ = rclcpp_action::create_client<ActionT>(this, action_name_);

    end_xyz_       = this->declare_parameter<std::vector<double>>("end_position", {0.40, 0.10, 0.25});
    traj_duration_ = this->declare_parameter<double>("traj_duration", 5.0);
    acc_duration_  = this->declare_parameter<double>("acc_duration", 0.5);
    traj_len_      = this->declare_parameter<int>("trajectory_len", 150);
    s_type_        = this->declare_parameter<std::string>("s_type", "trapezoidal");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(300),
      std::bind(&ExecuteLinearTrajClient::send_goal, this));

    rclcpp::on_shutdown([this]() {
    if (goal_handle_) {
        client_->async_cancel_goal(goal_handle_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
  }

private:
  void send_goal() {
    using namespace std::placeholders;
    RCLCPP_INFO(get_logger(), "Waiting for action server '%s'...", action_name_.c_str());
    client_->wait_for_action_server();  // attende finché il server non è up
    timer_->cancel();

    ActionT::Goal goal;
    goal.end_position.x = end_xyz_.at(0);
    goal.end_position.y = end_xyz_.at(1);
    goal.end_position.z = end_xyz_.at(2);
    goal.traj_duration  = traj_duration_;
    goal.acc_duration   = acc_duration_;
    goal.trajectory_len = traj_len_;
    goal.s_type         = s_type_;

    RCLCPP_INFO(get_logger(),
      "Send goal  '%s': p=[%.3f %.3f %.3f], T=%.3f, Ta=%.3f, N=%d, s=%s",
      action_name_.c_str(), goal.end_position.x, goal.end_position.y, goal.end_position.z,
      goal.traj_duration, goal.acc_duration, goal.trajectory_len, goal.s_type.c_str());

    rclcpp_action::Client<ActionT>::SendGoalOptions opts;
    opts.goal_response_callback =
      [this](GoalHandleT::SharedPtr gh){
        if (!gh) RCLCPP_WARN(this->get_logger(), "Goal REJECTED from server");
            else {
                goal_handle_ = gh; 
                RCLCPP_INFO(this->get_logger(), "Goal ACCEPTED: waitinf result...");
            }
        };

    opts.feedback_callback =
      [this](GoalHandleT::SharedPtr,
             const std::shared_ptr<const ActionT::Feedback> fb){
        if (fb->error_xyz.size() >= 3) {
          RCLCPP_INFO(this->get_logger(), "t=%.3f  err=[%.4f %.4f %.4f]",
            fb->elapsed_time, fb->error_xyz[0], fb->error_xyz[1], fb->error_xyz[2]);
        }
      };

    opts.result_callback =
      [this](const GoalHandleT::WrappedResult & res){
        using rclcpp_action::ResultCode;
        switch (res.code) {
          case ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "SUCCESS: final_error=%.6f", res.result->final_error_norm);
            break;
          case ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "ABORTED");
            break;
          case ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "CANCELED");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Result unkown");
            break;
        }
        rclcpp::shutdown();
      };

    (void)client_->async_send_goal(goal, opts);
  }

  // membri
  rclcpp_action::Client<ActionT>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<double> end_xyz_;
  double traj_duration_{}, acc_duration_{};
  int    traj_len_{};
  std::string s_type_;
  std::string action_name_;
  
};

} // namespace ros2_kdl_pkg

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_kdl_pkg::ExecuteLinearTrajClient)
