//
// Created by dj on 2025/12/10.
//

#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_TELEOP_KEYBOARD_H
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_TELEOP_KEYBOARD_H


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <iostream>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

class SentryTeleopKeyboard {
public:
    SentryTeleopKeyboard();
    ~SentryTeleopKeyboard();

    void run();

private:
    int getKey();
    void restoreTerminal();
    void publishVelocity(double vx, double vy, double omega);
    bool setGlobalMode(bool enable);

    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
	ros::ServiceClient reconfigure_client_;

    double linear_vel_;
    double angular_vel_;
    bool use_global_vel_;

    struct termios old_termios_;
    bool terminal_modified_;
    bool shutdown_requested_;
};

#endif //SENTRY_CHASSIS_CONTROLLER_SENTRY_TELEOP_KEYBOARD_H