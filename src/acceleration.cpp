//
// Created by dj on 2025/12/10.
//
#include <sentry_chassis_controller.h>

namespace sentry_chassis_controller {

    void SentryChassisController::applyAccelerationLimit(
    double &vx, double &vy, double &omega, const ros::Duration &period) {

        double dt = period.toSec();
        if (dt <= 0.0) {
            return; // 避免除以零
        }

        // 限制线加速度
        double dvx = vx - last_vx_;
        double dvy = vy - last_vy_;

        double max_dv = linear_acceleration_limit_ * dt;

        if (fabs(dvx) > max_dv) {
            dvx = (dvx > 0) ? max_dv : -max_dv;
        }
        if (fabs(dvy) > max_dv) {
            dvy = (dvy > 0) ? max_dv : -max_dv;
        }

        vx = last_vx_ + dvx;
        vy = last_vy_ + dvy;

        // 限制角加速度
        double domega = omega - last_omega_;
        double max_domega = angular_acceleration_limit_ * dt;

        if (fabs(domega) > max_domega) {
            domega = (domega > 0) ? max_domega : -max_domega;
        }

        omega = last_omega_ + domega;

        // 更新上一时刻速度
        last_vx_ = vx;
        last_vy_ = vy;
        last_omega_ = omega;
    }

}
