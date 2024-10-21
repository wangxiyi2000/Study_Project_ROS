/*
    需求: 实现基本的话题通信，一方发布数据，一方接收数据，
         实现的关键点:
         1.发送方
         2.接收方
         3.数据(此处为普通文本)

         PS: 二者需要设置相同的话题


    消息发布方:
        循环发布信息:HelloWorld 后缀数字编号

    实现流程:
        1.包含头文件 
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 ROS 句柄
        4.实例化 发布者 对象
        5.组织被发布的数据，并编写逻辑发布数据

*/
// 1.包含头文件 
#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息
#include <sstream>

int main(int argc, char  *argv[])
{   
    //设置编码
    setlocale(LC_ALL, "");

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    /*
        作用：ROS初始化函数

        参数：
                    1. argc         ---- 封装实参个数(n + 1)
                    2. argv         ---- 封装参数的数组
                    3. name      ---- 为节点命名（唯一性）  
                    4. options  ---- 节点启动选项

                    返回值：void

        使用：
                    1. argc 与 argv 的使用
                        如果按照ROS中的特定格式传入实参，那么ROS可以加以使用，比如用来设置全局参数、给节点重命名...
                    2.option的使用
                        节点名称需要保证唯一，会导致一个问题：同一个节点不能重复启动
                        结果：ROS 中当有重名的节点启动时，之前的节点会被关闭。
                        需求：特定场景下，需要一个节点多次启动且能正常运行，怎么办？
                        解决：设置启动项 ros::init_options::AnonymousName
                                    当创建ROS节点时，会在用户自定义的节点名称后缀随机数，从而避免重名问题
    */
    ros::init(argc,argv,"talker", ros::init_options::AnonymousName);
    
    //3.实例化 ROS 句柄；
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能

    //4.实例化 发布者 对象
    //泛型: 发布的消息类型
    //参数1: 要发布到的话题
    //参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    /*
        作用：创建发布者对象

        模板：被发布的消息的类型

        参数：
                    1.  话题名称
                    2.  队列长度
                    3.  latch(可选) 如果设置为 true， 会保存发布方的最后一条消息，并且
                        新的订阅对象连接到发布方时，发布者会将这条消息发送给订阅者
        使用：
                    latch   设置为  true 的作用?
                    以静态地图发布为例，方案1： 可以使用固定频率发送地图数据，但是效率低；方案2：可以将
                    地图发布对象的  latch   设置为 true ，并且发布方只发送一次数据，每当订阅者连接时，
                    将地图数据发送给订阅者（只发送一次），这样提高了数据的发送频率。
    */
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10, true);

    //5.组织被发布的数据，并编写逻辑发布数据
    //数据(动态组织)
    std_msgs::String msg;
    // msg.data = "你好啊！！！";
    std::string msg_front = "Hello 你好！"; //消息前缀
    int count = 0; //消息计数器

    //逻辑(一秒10次)
    ros::Rate r(1);

    //节点不死
    while (ros::ok())
    {
        //使用 stringstream 拼接字符串与编号
        std::stringstream ss;
        ss << msg_front << count;
        msg.data = ss.str();
        
        // if(count <= 10)
        // {
        //     //发布消息
        //     pub.publish(msg);
        //     //加入调试，打印发送的消息
        //     ROS_INFO("发送的消息:%s",msg.data.c_str());
        // }

        //发布消息
        pub.publish(msg);
        //加入调试，打印发送的消息
        ROS_INFO("发送的消息:%s",msg.data.c_str());

        //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        r.sleep();
        count++;//循环结束前，让 count 自增
        //暂无应用
        ros::spinOnce();
        ROS_INFO("一轮回调执行完毕...");
    }


    return 0;
}