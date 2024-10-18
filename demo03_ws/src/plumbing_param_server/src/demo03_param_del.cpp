#include "ros/ros.h"

/*
    演示参数删除：
    实现：
        ros::NodeHandle
            delParam("键")
            根据键删除参数，删除成功，返回 true，否则（参数不存在），返回 false

        ros::param
            del("键")
            根据键删除参数，删除成功，返回 true，否则（参数不存在），返回false
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "param_del_c");

    ros::NodeHandle nh;
    bool r1 = nh.deleteParam("nh_int");
    ROS_INFO("nh 删除结果: %d", r1);

    bool r2 = ros::param::del("param_int");
    ROS_INFO("param 删除结果: %d", r2);

    return 0;
}
