This is a repo for DXFinalProject.

# 舵轮底盘控制器

### 1.启动底盘控制器

```roslaunch sentry_chassis_controller sentry_chassis_controller.launch```

###  2.启动键盘控制节点

（可用官方的）

包里的（cpp）

```rosrun sentry_chassis_controller sentry_teleop_keyboard```

``````
w/s: 前进/后退
a/d: 左转/右转
q/e: 左平移/右平移
r/t: 左/右旋转
f: 切换全局模式 (底盘)
空格: 停止
ESC: 退出
``````

### 3.参数查看与调节

#### 动态参数

+ 四个轮子和舵机的PID

+ 速度模式：底盘/全局

+ 自锁功能

  ```rosrun rqt_reconfigure rqt_reconfigure ```

#### 配置文件参数

+ 底盘基本参数

+ 底盘加速度

  

