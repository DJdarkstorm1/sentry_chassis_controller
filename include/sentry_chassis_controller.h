//
// Created by dj on 2025/12/3.
//

#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

#include <sentry_chassis_controller/SentryChassisControllerConfig.h>


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
        //动态参数
        typedef dynamic_reconfigure::Server<SentryChassisControllerConfig> DynamicReconfigServer;
        boost::shared_ptr<DynamicReconfigServer> dyn_reconfig_server_;
        void dynReconfigCallback(SentryChassisControllerConfig &config,uint32_t level);


        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) ;
        double wheel_track_;
        double wheel_base_;
        double wheel_radius_;
        control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;
        control_toolbox::Pid pid_lf_wheel_, pid_rf_wheel_, pid_lb_wheel_, pid_rb_wheel_;

        ros::Subscriber cmd_vel_sub_;
        geometry_msgs::Twist cmd_vel_msg_;
        ros::Time last_cmd_vel_time_;
        double timeout_;
        bool cmd_vel_received_;

        //里程计
        ros::Publisher odom_pub_;
        tf::TransformBroadcaster tf_broadcaster_;
        nav_msgs::Odometry odom_msg_;
        ros::Time last_odom_update_time_;
        //里程计状态
        double x_, y_, theta_;
        double vx_, vy_, omega_;

        void updateOdometry(const ros::Time &time , const ros::Duration &period);
        void publishOdometry(const ros::Time &time);

        // 速度控制

        // 新增：速度模式参数
        bool use_global_vel_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // 新增：速度变换函数
        geometry_msgs::Twist transformVelocityToBaseLink(const geometry_msgs::Twist& vel_in_odom);

        // 自锁模式
        void enterLockMode(const ros::Time& time, const ros::Duration& period);
        // 新增：自锁参数
        double lock_angle_;      // 自锁角度（弧度）
        bool enable_lock_;       // 是否启用自锁
        // 自锁状态标志
        bool is_locked_;



    };
}// namespace sentry_chassis_controller

#endif //SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H