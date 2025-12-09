//
// Created by dj on 2025/12/5.
//
#include "sentry_chassis_controller.h"

namespace sentry_chassis_controller {

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

    // 新增：更新速度模式参数
    use_global_vel_ = config.use_global_vel;
    ROS_INFO("速度模式已更新：%s", use_global_vel_ ? "全局坐标系（odom）" : "底盘坐标系（base_link）");

        // 更新自锁参数
        lock_angle_ = config.lock_angle;
        enable_lock_ = config.enable_lock;
        ROS_INFO("自锁参数: 角度=%f rad (%f deg), 启用=%s",
                     lock_angle_, lock_angle_ * 180.0 / M_PI,
                     enable_lock_ ? "是" : "否");


    ROS_INFO("PID参数已更新: 转向(Kp=%f, Ki=%f, Kd=%f), 轮子(Kp=%f, Ki=%f, Kd=%f)",
             config.pivot_kp, config.pivot_ki, config.pivot_kd,
             config.wheel_kp, config.wheel_ki, config.wheel_kd);
}

}