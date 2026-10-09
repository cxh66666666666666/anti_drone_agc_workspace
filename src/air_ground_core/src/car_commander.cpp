// ============================================================================
// 【已废弃】car_commander —— 地面旧执行口
// 依据 M0 接口契约 v1.1：地面唯一执行口为 /agc/ground/cmd_vel，
// 由 ground_commander + ground_drive_adapter 实现（kinematic_sim 后端）。
// 本节点持续发布固定画圆速度，与唯一控制权冲突，故：
//   - 不再被任何 launch 启动；
//   - 已从 CMakeLists.txt 构建中移除；
//   - 源码仅作历史保留，代码本体不做修改。
// ============================================================================

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <clocale>

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    // 1. 初始化 ROS 节点，命名为 car_commander
    ros::init(argc, argv, "car_commander");
    ros::NodeHandle nh;

    // 2. 声明一个 Publisher，发布到 /cmd_vel 话题，队列长度设为 10
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 3. 设置循环频率为 10Hz (每秒发送10次控制指令)
    ros::Rate loop_rate(10); 

    ROS_INFO("空地协同: 移动平台 C++ 控制节点已启动，正在发送速度指令...");

    // 4. 控制主循环
    while (ros::ok()) {
        geometry_msgs::Twist msg;
        // 设定线速度为 0.5 m/s，角速度为 0.2 rad/s (画圆运动)
        msg.linear.x = 0.5; 
        msg.angular.z = 0.2; 
        
        // 发布消息
        vel_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep(); // 休眠以维持 10Hz 频率
    }

    return 0;
}