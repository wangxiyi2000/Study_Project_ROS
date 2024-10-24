/*
    注意命名空间的使用。
*/
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv,"haha");

    // // 方式一：需要创建节点句柄
    // ros::NodeHandle nh("turtlesim");
    // nh.setParam("background_r", 0);
    // nh.setParam("background_g", 0);
    // nh.setParam("background_b", 0);  

    // // 方式二：不需要创建节点句柄，调用API：ros::param
    // ros::param::set("/turtlesim/background_r", 0);
    // ros::param::set("/turtlesim/background_g", 100);
    // ros::param::set("/turtlesim/background_b", 0);
    
    //  方式三：需要创建节点句柄
    ros::NodeHandle nh;
    nh.setParam("/turltlesim/background_r", 0);
    nh.setParam("/turtlesim/background_g", 50);
    nh.setParam("/turtlesim/background_b", 100);
    
    return 0;
}
