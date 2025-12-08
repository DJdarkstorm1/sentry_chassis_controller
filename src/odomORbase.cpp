//
// Created by dj on 2025/12/6.
//
#include "sentry_chassis_controller.h"

namespace sentry_chassis_controller {
    // 新增：速度变换函数
    geometry_msgs::Twist SentryChassisController::transformVelocityToBaseLink(
        const geometry_msgs::Twist& vel_in_odom) {
        geometry_msgs::Twist vel_in_base = vel_in_odom;

        try {
            // 获取从odom到base_link的变换
            geometry_msgs::TransformStamped transform =
                tf_buffer_->lookupTransform("base_link", "odom", ros::Time(0));

            // 变换线速度
            geometry_msgs::Vector3Stamped vel_in, vel_out;
            vel_in.vector = vel_in_odom.linear;
            vel_in.header.frame_id = "odom";
            vel_in.header.stamp = ros::Time(0);

            tf2::doTransform(vel_in, vel_out, transform);
            vel_in_base.linear = vel_out.vector;

            // 角速度在平面运动下不变（假设绕z轴旋转）
            // 如果需要变换角速度，可以类似处理

            ROS_DEBUG_THROTTLE(1.0, "速度已从odom变换到底盘坐标系");

        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "TF变换失败：%s，使用原始速度", ex.what());
        }

        return vel_in_base;
    }

}
