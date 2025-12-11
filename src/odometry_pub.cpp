//
// Created by dj on 2025/12/5.
//
#include "sentry_chassis_controller.h"

namespace sentry_chassis_controller {

void SentryChassisController::updateOdometry(const ros::Time &time, const ros::Duration &period) {
    // 计算时间间隔
    double dt = period.toSec();
    if (dt <= 0.0) {
        return;
    }

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

    // 机器人中心的速度近似为四个轮子速度的平均值
    vx_ = (v1x + v2x + v3x + v4x) / 4.0;
    vy_ = (v1y + v2y + v3y + v4y) / 4.0;

    // 计算旋转角速度
    // 使用四个轮子的位置和速度来估计角速度
    double half_wheel_base = wheel_base_ / 2.0;//前后X方向
    double half_wheel_track = wheel_track_ / 2.0;//左右Y方向

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

    // 设置协方差简单估计
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
}
