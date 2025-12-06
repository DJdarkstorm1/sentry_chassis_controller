//
// Created by dj on 2025/12/3.
//
#include "sentry_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <ros/console.h>

namespace sentry_chassis_controller {
bool SentryChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                   ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    cmd_vel_received_ = false;
    last_cmd_vel_time_ =ros::Time::now();
    timeout_ = controller_nh.param("cmd_vel_timeout", 0.01);
    cmd_vel_sub_ =root_nh.subscribe<geometry_msgs::Twist>(
        "/cmd_vel",1,&SentryChassisController::cmdVelCallback,this);

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
  pid_lf_wheel_.initPid(3.0, 0.1, 0.0, 0.0, 0.0);
  pid_rf_wheel_.initPid(3.0, 0.1, 0.0, 0.0, 0.0);
  pid_lb_wheel_.initPid(3.0, 0.1, 0.0, 0.0, 0.0);
  pid_rb_wheel_.initPid(3.0, 0.1, 0.0, 0.0, 0.0);

//*******************************************************************************************
    dyn_reconfig_server_.reset(new DynamicReconfigServer(controller_nh));
    DynamicReconfigServer::CallbackType cb = boost::bind(&SentryChassisController::dynReconfigCallback,this,_1,_2);
    dyn_reconfig_server_->setCallback(cb);
//*****************************************************************************************

    cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>(
        "/cmd_vel",1,&SentryChassisController::cmdVelCallback,this);

    odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom",50);

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_link";

    last_odom_update_time_ = ros::Time::now();
    last_cmd_vel_time_ = ros::Time::now();

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
}

//***********************************************************************************************************
void SentryChassisController::dynReconfigCallback(SentryChassisControllerConfig &config, uint32_t level) {
    // 更新PID参数

    pid_lf_.setGains(config.pivot_kp, config.pivot_ki, config.pivot_kd, 0, 0);
    pid_rf_.setGains(config.pivot_kp, config.pivot_ki, config.pivot_kd, 0, 0);
    pid_lb_.setGains(config.pivot_kp, config.pivot_ki, config.pivot_kd, 0, 0);
    pid_rb_.setGains(config.pivot_kp, config.pivot_ki, config.pivot_kd, 0, 0);

    pid_lf_wheel_.setGains(config.wheel_kp, config.wheel_ki, config.wheel_kd, 0, 0);
    pid_rf_wheel_.setGains(config.wheel_kp, config.wheel_ki, config.wheel_kd, 0, 0);
    pid_lb_wheel_.setGains(config.wheel_kp, config.wheel_ki, config.wheel_kd, 0, 0);
    pid_rb_wheel_.setGains(config.wheel_kp, config.wheel_ki, config.wheel_kd, 0, 0);

    ROS_INFO("PID参数已更新: 转向(Kp=%f, Ki=%f, Kd=%f), 轮子(Kp=%f, Ki=%f, Kd=%f)",
             config.pivot_kp, config.pivot_ki, config.pivot_kd,
             config.wheel_kp, config.wheel_ki, config.wheel_kd);
}
//************************************************************************************************************

void SentryChassisController::updateOdometry(const ros::Time &time, const ros::Duration &period) {
    // 计算时间间隔
    double dt = period.toSec();
    if (dt <= 0.0) {
        return;
    }

    // 方法1：基于轮子实际速度的正运动学计算（推荐）
    // 获取轮子实际角速度（rad/s）
    double w1 = front_left_wheel_joint_.getVelocity();
    double w2 = front_right_wheel_joint_.getVelocity();
    double w3 = back_left_wheel_joint_.getVelocity();
    double w4 = back_right_wheel_joint_.getVelocity();

    // 获取轮子转向角度
    double theta1 = front_left_pivot_joint_.getPosition();
    double theta2 = front_right_pivot_joint_.getPosition();
    double theta3 = back_left_pivot_joint_.getPosition();
    double theta4 = back_right_pivot_joint_.getPosition();

    // 将角速度转换为线速度（m/s）
    double v1 = w1 * wheel_radius_;
    double v2 = w2 * wheel_radius_;
    double v3 = w3 * wheel_radius_;
    double v4 = w4 * wheel_radius_;

    // 计算每个轮子在机器人坐标系下的速度分量
    // 轮子速度向量在机器人坐标系下的投影
    double v1x = v1 * cos(theta1);
    double v1y = v1 * sin(theta1);

    double v2x = v2 * cos(theta2);
    double v2y = v2 * sin(theta2);

    double v3x = v3 * cos(theta3);
    double v3y = v3 * sin(theta3);

    double v4x = v4 * cos(theta4);
    double v4y = v4 * sin(theta4);

    // 方法A：简单平均法（适用于四轮速度相近的情况）
    // 机器人中心的速度近似为四个轮子速度的平均值
    vx_ = (v1x + v2x + v3x + v4x) / 4.0;
    vy_ = (v1y + v2y + v3y + v4y) / 4.0;

    // 方法B：使用运动学模型计算（更精确）
    // 对于四轮独立转向的机器人，可以使用逆运动学模型的逆运算
    // 这里使用简化的方法，假设机器人为刚体运动

    // 计算旋转角速度
    // 使用四个轮子的位置和速度来估计角速度
    double half_wheel_base = wheel_base_ / 2.0;
    double half_wheel_track = wheel_track_ / 2.0;

    // 计算每个轮子对旋转的贡献
    double omega1 = (v1x * (-half_wheel_track) + v1y * half_wheel_base) /
                    (half_wheel_base * half_wheel_base + half_wheel_track * half_wheel_track);
    double omega2 = (v2x * half_wheel_track + v2y * half_wheel_base) /
                    (half_wheel_base * half_wheel_base + half_wheel_track * half_wheel_track);
    double omega3 = (v3x * (-half_wheel_track) + v3y * (-half_wheel_base)) /
                    (half_wheel_base * half_wheel_base + half_wheel_track * half_wheel_track);
    double omega4 = (v4x * half_wheel_track + v4y * (-half_wheel_base)) /
                    (half_wheel_base * half_wheel_base + half_wheel_track * half_wheel_track);

    omega_ = (omega1 + omega2 + omega3 + omega4) / 4.0;

    // 积分得到位置和方向
    // 将机器人坐标系下的速度转换到世界坐标系
    double delta_x = (vx_ * cos(theta_) - vy_ * sin(theta_)) * dt;
    double delta_y = (vx_ * sin(theta_) + vy_ * cos(theta_)) * dt;
    double delta_theta = omega_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // 归一化角度到[-π, π]
    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    // 调试输出
    static int count = 0;
    if (count++ % 100 == 0) {
        ROS_INFO_STREAM_THROTTLE(1.0,
            "Odometry - position: x=" << x_ << ", y=" << y_ << ", theta=" << theta_
            << "\n velocity: vx=" << vx_ << ", vy=" << vy_ << ", omega=" << omega_
            << "\n wheel_velocity: w1=" << w1 << ", w2=" << w2 << ", w3=" << w3 << ", w4=" << w4);
    }
}

void SentryChassisController::publishOdometry(const ros::Time &time) {
    // 发布Odometry消息
    odom_msg_.header.stamp = time;

    // 设置位姿
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.position.z = 0.0;

    // 将偏航角转换为四元数
    tf::Quaternion q = tf::createQuaternionFromYaw(theta_);
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();

    // 设置协方差（这里使用简单估计）
    // 位置协方差
    for (int i = 0; i < 36; i++) {
        odom_msg_.pose.covariance[i] = 0.0;
    }
    odom_msg_.pose.covariance[0] = 0.01; // x
    odom_msg_.pose.covariance[7] = 0.01; // y
    odom_msg_.pose.covariance[35] = 0.01; // yaw

    // 设置速度
    odom_msg_.twist.twist.linear.x = vx_;
    odom_msg_.twist.twist.linear.y = vy_;
    odom_msg_.twist.twist.angular.z = omega_;

    // 速度协方差
    for (int i = 0; i < 36; i++) {
        odom_msg_.twist.covariance[i] = 0.0;
    }
    odom_msg_.twist.covariance[0] = 0.1; // vx
    odom_msg_.twist.covariance[7] = 0.1; // vy
    odom_msg_.twist.covariance[35] = 0.1; // omega

    // 发布消息
    odom_pub_.publish(odom_msg_);

    // 发布TF变换：odom -> base_link
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(odom_trans);
}

void SentryChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    cmd_vel_msg_ = *msg;
    cmd_vel_received_ = true;
    last_cmd_vel_time_ = ros::Time::now();

   /* ROS_DEBUG_STREAM_THROTTLE(1.0,"cmd_vel:  vx: "<<cmd_vel_msg_.linear.x
                                    <<",  vy: "<<cmd_vel_msg_.linear.y
                                    <<",  omega: "<<cmd_vel_msg_.angular.z);
*/
}

void SentryChassisController::update(const ros::Time &time, const ros::Duration &period) {

    if ((time-last_cmd_vel_time_).toSec()>timeout_) {
        cmd_vel_received_ = false;
        cmd_vel_msg_.linear.x = 0;
        cmd_vel_msg_.linear.y = 0;
        cmd_vel_msg_.angular.z = 0;
    }

    double vx = cmd_vel_msg_.linear.x;
    double vy = cmd_vel_msg_.linear.y;
    double omega = cmd_vel_msg_.angular.z;

    double dead_zone = 0.1;
    if (fabs(vx) < dead_zone) vx = 0.0;
    if (fabs(vy) < dead_zone) vy = 0.0;
    if (fabs(omega) < dead_zone) omega = 0.0;

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
