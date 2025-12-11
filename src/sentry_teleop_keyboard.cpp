//
// Created by dj on 2025/12/10.
//

#include "sentry_teleop_keyboard.h"
#include <unistd.h>
#include <signal.h>
#include <cmath>

SentryTeleopKeyboard::SentryTeleopKeyboard()
    : linear_vel_(0.5),
      angular_vel_(1.0),
      use_global_vel_(false),
      terminal_modified_(false),
      shutdown_requested_(false) {

    // 获取参数
    setlocale(LC_ALL,"");
    ros::NodeHandle private_nh("~");
    private_nh.param("linear_vel", linear_vel_, 0.5);
    private_nh.param("angular_vel", angular_vel_, 1.0);
    private_nh.param("use_global_vel", use_global_vel_, false);

    // 初始化发布器
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // 初始化动态参数服务客户端便于在键盘操作节点切换速度模式
    reconfigure_client_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(
        "/controller/sentry_chassis_controller/set_parameters");

    // 等待服务
    ros::Duration(1.0).sleep();
    if (reconfigure_client_.waitForExistence(ros::Duration(3.0))) {
        ROS_INFO("已连接到底盘控制器的动态参数服务");
    } else {
        ROS_WARN("无法连接到底盘控制器的动态参数服务，模式切换功能可能不可用");
    }

    // 保存终端设置
    tcgetattr(STDIN_FILENO, &old_termios_);

    std::cout << "\n键盘控制节点启动\n";
    std::cout << "w/s: 前进/后退\n";
    std::cout << "a/d: 左转/右转\n";
    std::cout << "q/e: 左平移/右平移\n";
    std::cout << "r/t: 左旋转/右旋转\n";
    std::cout << "f: 切换全局模式 (" << (use_global_vel_ ? "全局" : "底盘") << ")\n";
    std::cout << "空格: 停止\n";
    std::cout << "ESC: 退出\n";
}

SentryTeleopKeyboard::~SentryTeleopKeyboard() {
    // 停止运动
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 0;
    cmd_vel_pub_.publish(twist);

    restoreTerminal();
    std::cout << "\n节点关闭\n";
}

int SentryTeleopKeyboard::getKey() {
    struct termios new_termios;

    // 保存当前终端设置
    tcgetattr(STDIN_FILENO, &old_termios_);
    new_termios = old_termios_;
    terminal_modified_ = true;

    // 设置终端为原始模式
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 1;
    new_termios.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    int key = getchar();

    // 恢复终端设置
    restoreTerminal();

    return key;
}

void SentryTeleopKeyboard::restoreTerminal() {
    if (terminal_modified_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
        terminal_modified_ = false;
    }
}

void SentryTeleopKeyboard::publishVelocity(double vx, double vy, double omega) {
    geometry_msgs::Twist twist;
    twist.linear.x = vx;
    twist.linear.y = vy;
    twist.angular.z = omega;
    cmd_vel_pub_.publish(twist);
}

bool SentryTeleopKeyboard::setGlobalMode(bool enable) {
    if (!reconfigure_client_.exists()) {
        std::cout << "错误: 无法连接到底盘控制器的动态参数服务\n";
        return false;
    }

    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Config config;

    bool_param.name = "use_global_vel";
    bool_param.value = enable;
    config.bools.push_back(bool_param);
    srv.request.config = config;

    if (reconfigure_client_.call(srv)) {
        use_global_vel_ = enable;
        std::cout << "模式切换成功: " << (enable ? "全局模式" : "底盘模式") << "\n";
        return true;
    } else {
        std::cout << "错误: 无法设置模式参数\n";
        return false;
    }
}


void SentryTeleopKeyboard::run() {
    char key;

    while (ros::ok() && !shutdown_requested_) {
        key = getKey();

        switch (key) {
            case 27:  // ESC
                shutdown_requested_ = true;
                break;

            case ' ':  // 空格
                publishVelocity(0, 0, 0);
                std::cout << "停止\n";
                break;

            case 'f':
            case 'F':
                setGlobalMode(!use_global_vel_);
                break;

            case 'w':
            case 'W':
                publishVelocity(linear_vel_, 0, 0);
                std::cout << "前进\n";
                break;

            case 's':
            case 'S':
                publishVelocity(-linear_vel_, 0, 0);
                std::cout << "后退\n";
                break;

            case 'a':
            case 'A':
                publishVelocity(0, linear_vel_, 0);
                std::cout << "左平移\n";
                break;

            case 'd':
            case 'D':
                publishVelocity(0, -linear_vel_, 0);
                std::cout << "右平移\n";
                break;

            case 'q':
            case 'Q':
                publishVelocity(0, 0, angular_vel_);
                std::cout << "左转\n";
                break;

            case 'e':
            case 'E':
                publishVelocity(0, 0, -angular_vel_);
                std::cout << "右转\n";
                break;

            case 'r':
            case 'R':
                publishVelocity(0, 0, angular_vel_*10);
                std::cout << "左转小陀螺\n";
                break;

            case 't':
            case 'T':
                publishVelocity(0, 0, -angular_vel_*10);
                std::cout << "右转小陀螺\n";
                break;

            default:
                break;
        }

        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "sentry_teleop_keyboard");

    signal(SIGINT, [](int sig) {
        std::cout << "\n收到中断信号，退出\n";
        exit(0);
    });

    SentryTeleopKeyboard keyboard;
    keyboard.run();

    return 0;
}
