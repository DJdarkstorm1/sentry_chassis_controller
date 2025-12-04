#include "sentry_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace sentry_chassis_controller {

bool SentryChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                   ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  // 1. 绑定关节句柄（与URDF关节名一致）
  std::vector<std::string> wheel_joint_names = {
    "left_front_wheel_joint", "right_front_wheel_joint",
    "left_back_wheel_joint", "right_back_wheel_joint"
  };
  std::vector<std::string> pivot_joint_names = {
    "left_front_pivot_joint", "right_front_pivot_joint",
    "left_back_pivot_joint", "right_back_pivot_joint"
  };

  try {
    front_left_wheel_joint_ = effort_joint_interface->getHandle(wheel_joint_names[0]);
    front_right_wheel_joint_ = effort_joint_interface->getHandle(wheel_joint_names[1]);
    back_left_wheel_joint_ = effort_joint_interface->getHandle(wheel_joint_names[2]);
    back_right_wheel_joint_ = effort_joint_interface->getHandle(wheel_joint_names[3]);

    front_left_pivot_joint_ = effort_joint_interface->getHandle(pivot_joint_names[0]);
    front_right_pivot_joint_ = effort_joint_interface->getHandle(pivot_joint_names[1]);
    back_left_pivot_joint_ = effort_joint_interface->getHandle(pivot_joint_names[2]);
    back_right_pivot_joint_ = effort_joint_interface->getHandle(pivot_joint_names[3]);
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR("Failed to get joint handles: %s", e.what());
    return false;
  }

  // 2. 读取参数（支持从YAML配置文件加载）
  controller_nh.param("wheel_track", wheel_track_, 0.362);
  controller_nh.param("wheel_base", wheel_base_, 0.362);
  controller_nh.param("wheel_radius", wheel_radius_, 0.05);
  controller_nh.param("max_wheel_vel", max_wheel_vel_, 10.0);
  controller_nh.param("max_pivot_angle", max_pivot_angle_, M_PI/2);
  controller_nh.param("cmd_timeout", cmd_timeout_, 0.5);

  // 3. 初始化PID（可通过参数服务器调优）
  pid_lf_.init(ros::NodeHandle(controller_nh, "pivot_pid/left_front"), false);
  pid_rf_.init(ros::NodeHandle(controller_nh, "pivot_pid/right_front"), false);
  pid_lb_.init(ros::NodeHandle(controller_nh, "pivot_pid/left_back"), false);
  pid_rb_.init(ros::NodeHandle(controller_nh, "pivot_pid/right_back"), false);

  pid_lf_wheel_.init(ros::NodeHandle(controller_nh, "wheel_pid/left_front"), false);
  pid_rf_wheel_.init(ros::NodeHandle(controller_nh, "wheel_pid/right_front"), false);
  pid_lb_wheel_.init(ros::NodeHandle(controller_nh, "wheel_pid/left_back"), false);
  pid_rb_wheel_.init(ros::NodeHandle(controller_nh, "wheel_pid/right_back"), false);

  // 4. 订阅cmd_vel话题（Gazebo中通过rviz或终端发布指令）
  cmd_vel_sub_ = controller_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &SentryChassisController::cmdVelCallback, this);
  last_cmd_time_ = ros::Time::now();

  return true;
}

void SentryChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  cmd_vel_ = *msg;
  last_cmd_time_ = ros::Time::now();
}

void SentryChassisController::starting(const ros::Time& time) {
  // 启动时重置PID和指令
  pid_lf_.reset();
  pid_rf_.reset();
  pid_lb_.reset();
  pid_rb_.reset();
  pid_lf_wheel_.reset();
  pid_rf_wheel_.reset();
  pid_lb_wheel_.reset();
  pid_rb_wheel_.reset();
  cmd_vel_.linear.x = cmd_vel_.linear.y = cmd_vel_.angular.z = 0.0;
  last_cmd_time_ = time;
}

void SentryChassisController::stopping(const ros::Time& time) {
  // 停止时发送零指令
  front_left_wheel_joint_.setCommand(0.0);
  front_right_wheel_joint_.setCommand(0.0);
  back_left_wheel_joint_.setCommand(0.0);
  back_right_wheel_joint_.setCommand(0.0);
  front_left_pivot_joint_.setCommand(0.0);
  front_right_pivot_joint_.setCommand(0.0);
  back_left_pivot_joint_.setCommand(0.0);
  back_right_pivot_joint_.setCommand(0.0);
}

void SentryChassisController::update(const ros::Time& time, const ros::Duration& period) {
  // 1. 指令超时判断（无指令时停止运动）
  if ((time - last_cmd_time_).toSec() > cmd_timeout_) {
    cmd_vel_.linear.x = cmd_vel_.linear.y = cmd_vel_.angular.z = 0.0;
  }

  // 2. 全向运动正解（核心：根据cmd_vel计算各车轮转向角和速度）
  double vx = cmd_vel_.linear.x;    // x方向速度（前进/后退）
  double vy = cmd_vel_.linear.y;    // y方向速度（横移）
  double wz = cmd_vel_.angular.z;   // 角速度（旋转）

  // 车轮位置到旋转中心的距离（半轮距、半轴距）
  double b = wheel_base_ / 2.0;
  double d = wheel_track_ / 2.0;

  // 四个车轮的目标转向角和速度（LF:左前, RF:右前, LB:左后, RB:右后）
  double theta[4], wheel_vel[4];

  // 左前车轮
  theta[0] = atan2(vy + wz * b, vx - wz * d);
  wheel_vel[0] = sqrt(pow(vx - wz * d, 2) + pow(vy + wz * b, 2)) / wheel_radius_;

  // 右前车轮
  theta[1] = atan2(vy + wz * b, vx + wz * d);
  wheel_vel[1] = sqrt(pow(vx + wz * d, 2) + pow(vy + wz * b, 2)) / wheel_radius_;

  // 左后车轮
  theta[2] = atan2(vy - wz * b, vx - wz * d);
  wheel_vel[2] = sqrt(pow(vx - wz * d, 2) + pow(vy - wz * b, 2)) / wheel_radius_;

  // 右后车轮
  theta[3] = atan2(vy - wz * b, vx + wz * d);
  wheel_vel[3] = sqrt(pow(vx + wz * d, 2) + pow(vy - wz * b, 2)) / wheel_radius_;

  // 3. 限制最大角度和速度（避免超出硬件能力）
  for (int i = 0; i < 4; ++i) {
    theta[i] = std::min(std::max(theta[i], -max_pivot_angle_), max_pivot_angle_);
    wheel_vel[i] = std::min(std::max(wheel_vel[i], -max_wheel_vel_), max_wheel_vel_);
  }

  // 4. 转向关节PID控制（跟踪目标角度）
  front_left_pivot_joint_.setCommand(pid_lf_.computeCommand(theta[0] - front_left_pivot_joint_.getPosition(), period));
  front_right_pivot_joint_.setCommand(pid_rf_.computeCommand(theta[1] - front_right_pivot_joint_.getPosition(), period));
  back_left_pivot_joint_.setCommand(pid_lb_.computeCommand(theta[2] - back_left_pivot_joint_.getPosition(), period));
  back_right_pivot_joint_.setCommand(pid_rb_.computeCommand(theta[3] - back_right_pivot_joint_.getPosition(), period));

  // 5. 车轮速度PID控制（跟踪目标转速）
  front_left_wheel_joint_.setCommand(-pid_lf_wheel_.computeCommand(wheel_vel[0] - front_left_wheel_joint_.getVelocity(), period));
  front_right_wheel_joint_.setCommand(-pid_rf_wheel_.computeCommand(wheel_vel[1] - front_right_wheel_joint_.getVelocity(), period));
  back_left_wheel_joint_.setCommand(pid_lb_wheel_.computeCommand(-wheel_vel[2] - back_left_wheel_joint_.getVelocity(), period));
  back_right_wheel_joint_.setCommand(pid_rb_wheel_.computeCommand(-wheel_vel[3] - back_right_wheel_joint_.getVelocity(), period));
}

} // namespace sentry_chassis_controller

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
