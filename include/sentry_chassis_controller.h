//
// Created by dj on 2025/12/3.
//

#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <sentry_chassis_controller/SentryChassisControllerConfig.h>
#include <ros/callback_queue.h>

namespace sentry_chassis_controller {

    class SentryChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        SentryChassisController() = default;
        ~SentryChassisController() override = default;

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

        void starting(const ros::Time &time) override;
        void stopping(const ros::Time &time) override;

        double normalizeAngle(double target, double current) ;
    private:
        //****************************************************************************************
        typedef dynamic_reconfigure::Server<SentryChassisControllerConfig> DynamicReconfigServer;
        boost::shared_ptr<DynamicReconfigServer> dyn_reconfig_server_;


        void dynReconfigCallback(SentryChassisControllerConfig &config,uint32_t level);
//************************************************************************************************

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) ;
        double wheel_track_;
        double wheel_base_;
        double wheel_radius_;
        control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;
        control_toolbox::Pid pid_lf_wheel_, pid_rf_wheel_, pid_lb_wheel_, pid_rb_wheel_;

        ros::Subscriber cmd_vel_sub_;
        geometry_msgs::Twist cmd_vel_msg_;
        bool cmd_vel_received_;
        ros::Time last_cmd_vel_time_;
        double timeout_;


    };
}// namespace sentry_chassis_controller

#endif //SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H