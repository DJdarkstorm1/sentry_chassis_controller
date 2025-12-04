//
// Created by dj on 2025/11/28.
//

#ifndef SENTRY_CHASSIS_CONTROLLER_COPYTEST_H
#define SENTRY_CHASSIS_CONTROLLER_COPYTEST_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

namespace sentry_chassis_controller
{
class SentryChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
    public:
    SentryChassisController()=default;
    ~SentryChassisController()=default;

    bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
              ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    hardware_interface::JointHandle front_left_pivot_joint_,
                                    front_right_pivot_joint_,
                                    back_left_pivot_joint_,
                                    back_right_pivot_joint_;
    hardware_interface::JointHandle front_left_wheel_joint_,
                                    front_right_wheel_joint_,
                                    back_left_wheel_joint_,
                                    back_right_wheel_joint_;
    private:
        int state_{};
        ros::Time last_change_;
        double wheel_track_;//前后轮轴距离
        double wheel_base_;//左右轮轴距离
        double pivot_cmd_[6][4];//状态，轮子序号:左上为0号
        double wheel_cmd_[6][4];
        control_toolbox::Pid pid_lf_,
                             pid_rf_,
                             pid_lb_,
                             pid_rb_;
        control_toolbox::Pid pid_lf_wheel_,
                             pid_rf_wheel_,
                             pid_lb_wheel_,
                             pid_rb_wheel_;
};
}//namespace sentry_chassis_controller


#endif //SENTRY_CHASSIS_CONTROLLER_COPYTEST_H