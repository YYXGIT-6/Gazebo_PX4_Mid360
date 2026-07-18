#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <livox_ros_driver2/CustomPoint.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// 全局点云发布器
ros::Publisher pointcloud_pub;

/**
 * @brief Livox CustomMsg 转 ROS 标准 PointCloud2 核心函数
 * @param livox_msg Livox雷达的自定义消息指针
 * @return 转换后的ROS标准点云消息
 */
sensor_msgs::PointCloud2 livox2pointcloud(const livox_ros_driver2::CustomMsg::ConstPtr& livox_msg) {
    sensor_msgs::PointCloud2 cloud_msg;
    // 直接赋值头信息，简化代码且保证一致性
    cloud_msg.header = livox_msg->header;
    cloud_msg.height = 1;                // 无序点云，height设为1
    cloud_msg.width = livox_msg->points.size(); // 点云数量为宽度

    // 空点云校验：避免空遍历导致的内存问题，直接返回空点云
    if (cloud_msg.width == 0) {
        ROS_WARN_THROTTLE(1.0, "Livox message has no points, skip conversion!");
        return cloud_msg;
    }

    // 配置7个字段：offset_time/x/y/z/intensity/tag/line（适配ROS Noetic，逐字段赋值）
    cloud_msg.fields.resize(7);
    // 字段0：offset_time - 时间偏移，UINT32，偏移0字节
    cloud_msg.fields[0].name = "offset_time";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::PointField::UINT32;
    cloud_msg.fields[0].count = 1;
    // 字段1：x - X坐标，FLOAT32，偏移4字节
    cloud_msg.fields[1].name = "x";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;
    // 字段2：y - Y坐标，FLOAT32，偏移8字节
    cloud_msg.fields[2].name = "y";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;
    // 字段3：z - Z坐标，FLOAT32，偏移12字节
    cloud_msg.fields[3].name = "z";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[3].count = 1;
    // 字段4：intensity - 点云强度，FLOAT32，偏移16字节
    cloud_msg.fields[4].name = "intensity";
    cloud_msg.fields[4].offset = 16;
    cloud_msg.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[4].count = 1;
    // 字段5：tag - Livox标签，UINT8，偏移20字节
    cloud_msg.fields[5].name = "tag";
    cloud_msg.fields[5].offset = 20;
    cloud_msg.fields[5].datatype = sensor_msgs::PointField::UINT8;
    cloud_msg.fields[5].count = 1;
    // 字段6：line - 激光线号，UINT8，偏移21字节
    cloud_msg.fields[6].name = "line";
    cloud_msg.fields[6].offset = 21;
    cloud_msg.fields[6].datatype = sensor_msgs::PointField::UINT8;
    cloud_msg.fields[6].count = 1;

    // 核心修复：设置实际的点步长（手动配置字段总字节数22），彻底解决内存越界
    cloud_msg.point_step = 22;
    // 行步长 = 点数量 * 点步长，按实际内存需求计算
    cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
    // 按实际需要的内存大小分配数据区，无冗余/不足
    cloud_msg.data.resize(cloud_msg.row_step);

    // ROS规范：显式设置字节序和点云密集性（避免潜在解析问题）
    cloud_msg.is_bigendian = false; // 主流硬件均为小端序
    cloud_msg.is_dense = true;     // 若点云无nan/inf可设为true，通用设为false

    // 初始化PointCloud2迭代器，绑定对应字段
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_offset_time(cloud_msg, "offset_time");
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_tag(cloud_msg, "tag");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_line(cloud_msg, "line");

    // 遍历Livox点云，逐点赋值到ROS点云
    for (const auto& livox_p : livox_msg->points) {
        *iter_offset_time = livox_p.offset_time;
        *iter_x = livox_p.x;
        *iter_y = livox_p.y;
        *iter_z = livox_p.z;
        *iter_intensity = livox_p.reflectivity;
        *iter_tag = livox_p.tag;
        *iter_line = livox_p.line;

        // 迭代器自增，指向下一个点的对应字段
        ++iter_offset_time;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
        ++iter_tag;
        ++iter_line;
    }

    return cloud_msg;
}

/**
 * @brief Livox自定义消息的回调函数
 * @param livox_msg 订阅到的Livox雷达消息
 */
void customMsgCallback(const livox_ros_driver2::CustomMsg::ConstPtr& livox_msg) {
    // 转换并发布点云
    pointcloud_pub.publish(livox2pointcloud(livox_msg));
    ROS_INFO_STREAM("Converted livox_ros_driver2::CustomMsg to sensor_msgs::PointCloud2");
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "livox2pointcloud_node");
    // 私有节点句柄：仅读取本节点的参数，避免命名冲突
    ros::NodeHandle nh("~");

    // 从参数服务器读取配置，无配置则使用默认值（你的默认订阅话题/scan）
    std::string pointcloud_topic, livox_topic;
    nh.param<std::string>("pointcloud_topic", pointcloud_topic, "/livox/pointcloud2");
    nh.param<std::string>("livox_topic", livox_topic, "/scan");

    // 创建点云发布器：队列大小100（适配雷达10Hz频率，避免消息堆积）
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 100);
    // 创建Livox消息订阅器：队列大小100，与发布器匹配
    auto custom_msg_sub = nh.subscribe<livox_ros_driver2::CustomMsg>(livox_topic, 100, customMsgCallback);

    // 核心修复：仅保留一个ROS消息循环，阻塞等待并处理消息
    ros::spin();

    return 0;
}
