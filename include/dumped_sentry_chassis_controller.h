#ifndef SENTRY_CHASSIS_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>

namespace sentry_chassis_controller {

class SentryChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:
  // 关节句柄
  hardware_interface::JointHandle front_left_wheel_joint_;
  hardware_interface::JointHandle front_right_wheel_joint_;
  hardware_interface::JointHandle back_left_wheel_joint_;
  hardware_interface::JointHandle back_right_wheel_joint_;
  hardware_interface::JointHandle front_left_pivot_joint_;
  hardware_interface::JointHandle front_right_pivot_joint_;
  hardware_interface::JointHandle back_left_pivot_joint_;
  hardware_interface::JointHandle back_right_pivot_joint_;

  // PID控制器
  control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;
  control_toolbox::Pid pid_lf_wheel_, pid_rf_wheel_, pid_lb_wheel_, pid_rb_wheel_;

  // 底盘参数
  double wheel_track_;    // 轮距（m）
  double wheel_base_;     // 轴距（m）
  double wheel_radius_;   // 车轮半径（m）
  double max_wheel_vel_;  // 最大车轮转速（rad/s）
  double max_pivot_angle_;// 最大转向角度（rad）

  // 速度指令
  geometry_msgs::Twist cmd_vel_;
  ros::Subscriber cmd_vel_sub_;
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  // 辅助变量
  ros::Time last_cmd_time_;
  double cmd_timeout_;    // 指令超时时间（s）
};

} // namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_H
