//
// Created by dj on 2025/12/3.
//
#include "sentry_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <ros/console.h>

namespace sentry_chassis_controller {
bool SentryChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                   ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    setlocale(LC_ALL, "");
  front_left_wheel_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_wheel_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_wheel_joint_ =
      effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_wheel_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");


  front_left_pivot_joint_ =
      effort_joint_interface->getHandle("left_front_pivot_joint");
  front_right_pivot_joint_ =
      effort_joint_interface->getHandle("right_front_pivot_joint");
  back_left_pivot_joint_ =
      effort_joint_interface->getHandle("left_back_pivot_joint");
  back_right_pivot_joint_ =
      effort_joint_interface->getHandle("right_back_pivot_joint");

  wheel_track_ = controller_nh.param("wheel_track", 0.362);
  wheel_base_ = controller_nh.param("wheel_base", 0.362);
  wheel_radius_ = controller_nh.param("wheel_radius", 0.05);

  pid_lf_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);
  pid_rf_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);
  pid_lb_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);
  pid_rb_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);

  // PID for wheel velocity control
  pid_lf_wheel_.initPid(100.0, 0.0, 1.0, 0.0, 0.0);
  pid_rf_wheel_.initPid(100.0, 0.0, 1.0, 0.0, 0.0);
  pid_lb_wheel_.initPid(100.0, 0.0, 1.0, 0.0, 0.0);
  pid_rb_wheel_.initPid(100.0, 0.0, 1.0, 0.0, 0.0);

    //动态参数
    dyn_reconfig_server_.reset(new DynamicReconfigServer(controller_nh));
    DynamicReconfigServer::CallbackType cb = boost::bind(&SentryChassisController::dynReconfigCallback,this,_1,_2);
    dyn_reconfig_server_->setCallback(cb);

    //速度控制
    cmd_vel_received_ = false;
    last_cmd_vel_time_ =ros::Time::now();
    timeout_ = controller_nh.param("cmd_vel_timeout", 0.5);//超时停车
    cmd_vel_sub_ =root_nh.subscribe<geometry_msgs::Twist>(
        "/cmd_vel",10,&SentryChassisController::cmdVelCallback,this);

    is_locked_ = false;  // 初始不处于自锁状态
    //自锁角度参数
    lock_angle_ = controller_nh.param("lock_angle", M_PI / 4.0);
    enable_lock_ = controller_nh.param("enable_lock", true);

    //读取速度模式参数
    use_global_vel_ = controller_nh.param("use_global_vel", false);
    if (use_global_vel_) {
        ROS_INFO("速度模式：全局坐标系（odom）");
    } else {
        ROS_INFO("速度模式：底盘坐标系（base_link）");
    }

    // 全局/底盘速度模式：初始化tf2 Buffer和Listener
    tf_buffer_.reset(new tf2_ros::Buffer());
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

    //里程计
    odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom",10);
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_link";
    last_odom_update_time_ = ros::Time::now();

    //加速度
    last_vx_ = 0.0;
    last_vy_ = 0.0;
    last_omega_ = 0.0;
    // 读取加速度限制参数
    linear_acceleration_limit_ = controller_nh.param("linear_acceleration_limit", 2.0);
    angular_acceleration_limit_ = controller_nh.param("angular_acceleration_limit", 1.0);

    return true;
}

void SentryChassisController::starting(const ros::Time &time) {
    pid_lf_.reset();
    pid_rf_.reset();
    pid_lb_.reset();
    pid_rb_.reset();

    pid_lf_wheel_.reset();
    pid_rf_wheel_.reset();
    pid_lb_wheel_.reset();
    pid_rb_wheel_.reset();

    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.angular.z = 0.0;

    x_ = y_ = theta_ = 0.0;
    vx_ = vy_ = omega_ = 0.0;
    last_odom_update_time_ = time;
    is_locked_ = false;  // 启动时不处于自锁状态
}

void SentryChassisController::stopping(const ros::Time &time) {
    front_left_wheel_joint_.setCommand(0.0);
    front_right_wheel_joint_.setCommand(0.0);
    back_left_wheel_joint_.setCommand(0.0);
    back_right_wheel_joint_.setCommand(0.0);

    front_left_pivot_joint_.setCommand(0.0);
    front_right_pivot_joint_.setCommand(0.0);
    back_left_pivot_joint_.setCommand(0.0);
    back_right_pivot_joint_.setCommand(0.0);

    is_locked_ = false;

}


void SentryChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    cmd_vel_msg_ = *msg;
    cmd_vel_received_ = true;
    last_cmd_vel_time_ = ros::Time::now();

    // 收到新命令时解除自锁状态
    if (is_locked_) {
        ROS_INFO("收到新的速度命令，自动解除自锁状态");
        is_locked_ = false;
    }


    ROS_DEBUG_STREAM_THROTTLE(1.0,"cmd_vel:  vx: "<<cmd_vel_msg_.linear.x
                                    <<",  vy: "<<cmd_vel_msg_.linear.y
                                    <<",  omega: "<<cmd_vel_msg_.angular.z);

}

void SentryChassisController::update(const ros::Time &time, const ros::Duration &period) {

  // 检查是否超时
    if ((time - last_cmd_vel_time_).toSec() > timeout_) {
        cmd_vel_received_ = false;
        cmd_vel_msg_.linear.x = 0;
        cmd_vel_msg_.linear.y = 0;
        cmd_vel_msg_.angular.z = 0;

        // 如果启用了自锁功能，进入自锁模式
        if (enable_lock_) {
            // 进入自锁模式
            if (!is_locked_) {
                ROS_DEBUG_THROTTLE(1.0, "进入自锁模式");
                is_locked_ = true;
            }

            // 获取当前角度
            double current_theta_lf = front_left_pivot_joint_.getPosition();
            double current_theta_rf = front_right_pivot_joint_.getPosition();
            double current_theta_lb = back_left_pivot_joint_.getPosition();
            double current_theta_rb = back_right_pivot_joint_.getPosition();

            // 设置自锁角度：形成X型锁定
            double target_theta_lf = lock_angle_;
            double target_theta_rf = -lock_angle_;
            double target_theta_lb = -lock_angle_;
            double target_theta_rb = lock_angle_;

            // 归一化角度
            target_theta_lf = normalizeAngle(target_theta_lf, current_theta_lf);
            target_theta_rf = normalizeAngle(target_theta_rf, current_theta_rf);
            target_theta_lb = normalizeAngle(target_theta_lb, current_theta_lb);
            target_theta_rb = normalizeAngle(target_theta_rb, current_theta_rb);

            // 停止所有轮子
            front_left_wheel_joint_.setCommand(0.0);
            front_right_wheel_joint_.setCommand(0.0);
            back_left_wheel_joint_.setCommand(0.0);
            back_right_wheel_joint_.setCommand(0.0);

            // 设置转向角度到自锁位置
            front_left_pivot_joint_.setCommand(
                pid_lf_.computeCommand(target_theta_lf - current_theta_lf, period));
            front_right_pivot_joint_.setCommand(
                pid_rf_.computeCommand(target_theta_rf - current_theta_rf, period));
            back_left_pivot_joint_.setCommand(
                pid_lb_.computeCommand(target_theta_lb - current_theta_lb, period));
            back_right_pivot_joint_.setCommand(
                pid_rb_.computeCommand(target_theta_rb - current_theta_rb, period));
        } else {
            // 如果不启用自锁，只是停止轮子（保持原有行为）
            front_right_wheel_joint_.setCommand(0.0);
            front_left_wheel_joint_.setCommand(0.0);
            back_left_wheel_joint_.setCommand(0.0);
            back_right_wheel_joint_.setCommand(0.0);

            // 转向电机可以保持当前位置，不需要额外设置
            front_left_pivot_joint_.setCommand(0.0);
            front_right_pivot_joint_.setCommand(0.0);
            back_left_pivot_joint_.setCommand(0.0);
            back_right_pivot_joint_.setCommand(0.0);

            // 退出自锁状态（如果之前处于自锁状态）
            if (is_locked_) {
                is_locked_ = false;
            }
        }

        updateOdometry(time, period);
        publishOdometry(time);
        return;
    } else {
        // 如果收到新命令，退出自锁状态
        if (is_locked_) {
            is_locked_ = false;
            ROS_DEBUG_THROTTLE(1.0, "退出自锁模式");
        }
    }

    double vx = cmd_vel_msg_.linear.x;
    double vy = cmd_vel_msg_.linear.y;
    double omega = cmd_vel_msg_.angular.z;

    //死区
    double dead_zone = 0.1;
    if (fabs(vx) < dead_zone) vx = 0.0;
    if (fabs(vy) < dead_zone) vy = 0.0;
    if (fabs(omega) < dead_zone) omega = 0.0;

    //如果使用全局坐标系速度模式，则进行坐标变换
    if (use_global_vel_) {
        geometry_msgs::Twist vel_odom, vel_base;
        vel_odom.linear.x = vx;
        vel_odom.linear.y = vy;
        vel_odom.angular.z = omega;

        vel_base = transformVelocityToBaseLink(vel_odom);

        vx = vel_base.linear.x;
        vy = vel_base.linear.y;
        omega = vel_base.angular.z;

        // 再次死区处理
        if (fabs(vx) < dead_zone) vx = 0.0;
        if (fabs(vy) < dead_zone) vy = 0.0;
        if (fabs(omega) < dead_zone) omega = 0.0;
    }

    // 应用加速度限制
    applyAccelerationLimit(vx, vy, omega, period);

    if (fabs(vx) < 0.01 && fabs(vy) < 0.01 && fabs(omega) < 0.01) {
        front_right_wheel_joint_.setCommand(0.0);
        front_left_wheel_joint_.setCommand(0.0);
        back_left_wheel_joint_.setCommand(0.0);
        back_right_wheel_joint_.setCommand(0.0);

        updateOdometry(time, period);
        publishOdometry(time);

        return;
    }

    static int counter = 0;
    if (counter++ % 100 == 0) {
        ROS_INFO_STREAM_THROTTLE(1.0,"Cmd_vel   vx:" << vx << ",   vy:" << vy << ",   omega:" << omega);
        ROS_INFO_STREAM_THROTTLE(1.0,"get_Cmd_vel  \n wheel_lf_speed: " << front_left_wheel_joint_.getVelocity()*wheel_radius_<<
                                     ",\n  lf_pivot_theta: " <<front_left_pivot_joint_.getPosition()<<
                                     ",\n  lf_pivot_omega: " <<front_left_pivot_joint_.getVelocity());
    }

    // 判断运动模式
    bool is_straight_line = (fabs(vy) < 0.01 && fabs(omega) < 0.01); // 纯前后直线运动
    bool is_sideways = (fabs(vx) < 0.01 && fabs(omega) < 0.01); // 纯左右平移
    bool is_rotation = (fabs(vx) < 0.01 && fabs(vy) < 0.01); // 纯旋转

    // 角度和速度初始化
  double theta_lf = 0.0, v_lf = 0.0;
  double theta_rf = 0.0, v_rf = 0.0;
  double theta_lb = 0.0, v_lb = 0.0;
  double theta_rb = 0.0, v_rb = 0.0;

  if (is_straight_line) {
    // 纯前后直线运动：所有舵机角度为0，速度相同
    double wheel_speed_ = vx / wheel_radius_; // 转换为角速度

    theta_lf = 0.0;
    theta_rf = 0.0;
    theta_lb = 0.0;
    theta_rb = 0.0;

    v_lf = wheel_speed_;
    v_rf = wheel_speed_;
    v_lb = wheel_speed_;
    v_rb = wheel_speed_;


  } else if (is_sideways) {
    // 纯左右平移：所有舵机角度为90度，速度正负改变方向
    double wheel_speed_ = vy / wheel_radius_;

      // 向左右平移
      theta_lf = M_PI/2;
      theta_rf = M_PI/2;
      theta_lb = M_PI/2;
      theta_rb = M_PI/2;

    v_lf = wheel_speed_;
    v_rf = wheel_speed_;
    v_lb = wheel_speed_;
    v_rb = wheel_speed_;

  } else if (is_rotation) {
    // 纯旋转（原地转向）：使用运动学模型计算
    double half_wheel_base = wheel_base_ / 2.0;
    double half_wheel_track = wheel_track_ / 2.0;

    // 计算每个轮子的速度
    double radius = sqrt(half_wheel_base * half_wheel_base + half_wheel_track * half_wheel_track);
    double linear_speed = omega * radius; // 线速度
    double wheel_speed_ = linear_speed / wheel_radius_;

    // 计算角度：每个轮子指向圆心
    theta_lf = atan2(half_wheel_base, -half_wheel_track);
    theta_rf = atan2(half_wheel_base, half_wheel_track);
    theta_lb = atan2(-half_wheel_base, -half_wheel_track);
    theta_rb = atan2(-half_wheel_base, half_wheel_track);

    v_lf = wheel_speed_;
    v_rf = wheel_speed_;
    v_lb = wheel_speed_;
    v_rb = wheel_speed_;

  } else {
    // 组合运动：使用完整的运动学模型
    double half_wheel_base = wheel_base_ / 2.0;
    double half_wheel_track = wheel_track_ / 2.0;

    // 左前轮
    double vx_lf = vx - omega * half_wheel_track;
    double vy_lf = vy + omega * half_wheel_base;
    double linear_speed_lf = sqrt(vx_lf * vx_lf + vy_lf * vy_lf);
    theta_lf = atan2(vy_lf, vx_lf);
    v_lf = linear_speed_lf / wheel_radius_;

    // 右前轮
    double vx_rf = vx + omega * half_wheel_track;
    double vy_rf = vy + omega * half_wheel_base;
    double linear_speed_rf = sqrt(vx_rf * vx_rf + vy_rf * vy_rf);
    theta_rf = atan2(vy_rf, vx_rf);
    v_rf = linear_speed_rf / wheel_radius_;

    // 左后轮
    double vx_lb = vx - omega * half_wheel_track;
    double vy_lb = vy - omega * half_wheel_base;
    double linear_speed_lb = sqrt(vx_lb * vx_lb + vy_lb * vy_lb);
    theta_lb = atan2(vy_lb, vx_lb);
    v_lb = linear_speed_lb / wheel_radius_;

    // 右后轮
    double vx_rb = vx + omega * half_wheel_track;
    double vy_rb = vy - omega * half_wheel_base;
    double linear_speed_rb = sqrt(vx_rb * vx_rb + vy_rb * vy_rb);
    theta_rb = atan2(vy_rb, vx_rb);
    v_rb = linear_speed_rb / wheel_radius_;
  }

  // 获取当前角度
  double current_theta_lf = front_left_pivot_joint_.getPosition();
  double current_theta_rf = front_right_pivot_joint_.getPosition();
  double current_theta_lb = back_left_pivot_joint_.getPosition();
  double current_theta_rb = back_right_pivot_joint_.getPosition();

  // 角度归一化（保持原有函数）
  theta_lf = normalizeAngle(theta_lf, current_theta_lf);
  theta_rf = normalizeAngle(theta_rf, current_theta_rf);
  theta_lb = normalizeAngle(theta_lb, current_theta_lb);
  theta_rb = normalizeAngle(theta_rb, current_theta_rb);

  // 设置轮子速度
  front_left_wheel_joint_.setCommand(pid_lf_wheel_.computeCommand(v_lf - front_left_wheel_joint_.getVelocity(), period));
  front_right_wheel_joint_.setCommand(pid_rf_wheel_.computeCommand(v_rf - front_right_wheel_joint_.getVelocity(), period));
  back_left_wheel_joint_.setCommand(pid_lb_wheel_.computeCommand(v_lb - back_left_wheel_joint_.getVelocity(), period));
  back_right_wheel_joint_.setCommand(pid_rb_wheel_.computeCommand(v_rb - back_right_wheel_joint_.getVelocity(), period));

  // 设置转向角度
  front_left_pivot_joint_.setCommand(pid_lf_.computeCommand(theta_lf - current_theta_lf, period));
  front_right_pivot_joint_.setCommand(pid_rf_.computeCommand(theta_rf - current_theta_rf, period));
  back_left_pivot_joint_.setCommand(pid_lb_.computeCommand(theta_lb - current_theta_lb, period));
  back_right_pivot_joint_.setCommand(pid_rb_.computeCommand(theta_rb - current_theta_rb, period));

    updateOdometry(time, period);
    publishOdometry(time);
}
double SentryChassisController::normalizeAngle(double target, double current) {
    while (target > M_PI) target -= 2 * M_PI;
    while (target < -M_PI) target += 2 * M_PI;

    double diff = target-current;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;

    return current+diff;
}
PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
}
