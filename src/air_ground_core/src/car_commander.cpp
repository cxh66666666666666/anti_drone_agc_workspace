#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {
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