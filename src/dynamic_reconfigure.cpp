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

    ROS_INFO("PID参数已更新: 转向(Kp=%f, Ki=%f, Kd=%f), 轮子(Kp=%f, Ki=%f, Kd=%f)",
             config.pivot_kp, config.pivot_ki, config.pivot_kd,
             config.wheel_kp, config.wheel_ki, config.wheel_kd);
}

}