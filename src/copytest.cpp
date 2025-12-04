//
// Created by dj on 2025/11/28.
//
#include "sentry_chassis_controller/copytest.h"
#include <pluginlib/class_list_macros.h>

namespace sentry_chassis_controller
{
    bool SentryChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                        ros::NodeHandle &root_nh,ros::NodeHandle &controller_nh)
    {
        front_left_wheel_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_wheel_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_wheel_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_wheel_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        front_left_pivot_joint_ = effort_joint_interface->getHandle("left_front_pivot_joint");
        front_right_pivot_joint_ = effort_joint_interface->getHandle("right_front_pivot_joint");
        back_left_pivot_joint_ = effort_joint_interface->getHandle("left_back_pivot_joint");
        back_right_pivot_joint_ = effort_joint_interface->getHandle("right_back_pivot_joint");

        wheel_track_ = controller_nh.param("wheel_track", 0.362);
        wheel_base_ = controller_nh.param("wheel_base", 0.362);

        pid_lf_.initPid(1.0,0.0,0.0,0.0,0.0);
        pid_rf_.initPid(1.0,0.0,0.0,0.0,0.0);
        pid_lb_.initPid(1.0,0.0,0.0,0.0,0.0);
        pid_rb_.initPid(1.0,0.0,0.0,0.0,0.0);

        pid_lf_wheel_.initPid(2.0,0.1,0.0,0.0,0.0);
        pid_rf_wheel_.initPid(2.0,0.1,0.0,0.0,0.0);
        pid_lb_wheel_.initPid(2.0,0.1,0.0,0.0,0.0);
        pid_rb_wheel_.initPid(2.0,0.1,0.0,0.0,0.0);

        double speed = 8.0;
        //向前
        double theta_fwd[4] = {0.0,0.0,0.0,0.0};
        double v_fwd[4] = {speed,speed,speed,speed};

        //向后
        double theta_bwd[4] = {0.0,0.0,0.0,0.0};
        double v_bwd[4] = {-speed,-speed,-speed,-speed};

        //向左
        double theta_left[4] = {M_PI/2,M_PI / 2,M_PI / 2,M_PI / 2};
        double v_left[4] = {speed,speed,speed,speed};

        //向右
        double theta_right[4] = {-M_PI/2,-M_PI / 2,-M_PI / 2,-M_PI / 2};
        double v_right[4] = {speed,speed,speed,speed};

        //右陀螺
        double theta_right_revolve[4] = {-M_PI / 4,M_PI / 4,-M_PI / 4,M_PI / 4};
        double v_right_revolve[4] = {-speed,-speed,speed,speed};

        //左陀螺
        double theta_left_revolve[4] = {-M_PI / 4,M_PI / 4,-M_PI / 4,M_PI / 4};
        double v_left_revolve[4] = {speed,speed,-speed,-speed};

        for (int i = 0; i < 4; i++)
        {
            pivot_cmd_[0][i] = theta_fwd[i];
            wheel_cmd_[0][i] = v_fwd[i];
            pivot_cmd_[1][i] = theta_bwd[i];
            wheel_cmd_[1][i] = v_bwd[i];
            pivot_cmd_[2][i] = theta_left[i];
            wheel_cmd_[2][i] = v_left[i];
            pivot_cmd_[3][i] = theta_right[i];
            wheel_cmd_[3][i] = v_right[i];

            pivot_cmd_[4][i] = theta_right_revolve[i];
            wheel_cmd_[4][i] = v_right_revolve[i];
            pivot_cmd_[5][i] = theta_left_revolve[i];
            wheel_cmd_[5][i] = v_left_revolve[i];
        }
        return true;
    }
    void SentryChassisController::update(const ros::Time& time, const ros::Duration& period)
    {
        if ((time - last_change_).toSec()> 8)
        {
            state_ = (state_ + 1) % 6;
            last_change_ = time;
        }

        front_left_wheel_joint_.setCommand(
            pid_lf_wheel_.computeCommand(wheel_cmd_[state_][0] - front_left_wheel_joint_.getVelocity(),period));
        front_right_wheel_joint_.setCommand(
            pid_rf_wheel_.computeCommand(wheel_cmd_[state_][1] - front_right_wheel_joint_.getVelocity(),period));
        back_left_wheel_joint_.setCommand(
            pid_lb_wheel_.computeCommand(wheel_cmd_[state_][2] - back_left_wheel_joint_.getVelocity(),period));
        back_right_wheel_joint_.setCommand(
            pid_rb_wheel_.computeCommand(wheel_cmd_[state_][3] - back_right_wheel_joint_.getVelocity(),period));

        front_left_pivot_joint_.setCommand(
            pid_lf_.computeCommand(pivot_cmd_[state_][0] - front_left_pivot_joint_.getVelocity(),period));
        front_right_pivot_joint_.setCommand(
            pid_rf_.computeCommand(pivot_cmd_[state_][1] - front_right_pivot_joint_.getVelocity(),period));
        back_left_pivot_joint_.setCommand(
            pid_lb_.computeCommand(pivot_cmd_[state_][2] - back_left_pivot_joint_.getVelocity(),period));
        back_right_pivot_joint_.setCommand(
            pid_rb_.computeCommand(pivot_cmd_[state_][3] - back_right_pivot_joint_.getVelocity(),period));
    }
PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController,controller_interface::ControllerBase)
}
