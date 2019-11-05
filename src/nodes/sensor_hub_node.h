#ifndef HONGO_SENSOR_HUB_SENSOR_HUB_NODE_H
#define HONGO_SENSOR_HUB_SENSOR_HUB_NODE_H

#include <thread>
#include <string.h>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_hub/SensorHubConfig.h>

#include "hongo_msgs/default_topics.h"
#include "sensor_hub/sensor_hub.h"

namespace sensor_hub
{

class SensorReadNode
{
public:
    SensorReadNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~SensorReadNode();

    void readSensorData();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Publisher range_pub_;
    ros::Publisher focus_pub_;
    ros::Publisher load_pub_;

    SensorHub sensor_hub_;

    sensor_msgs::Range range_msg_;
    std_msgs::UInt8 focus_msg_;
    geometry_msgs::WrenchStamped load_msg_;
};

class SensorWriteNode
{
public:
    SensorWriteNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~SensorWriteNode();

    void SensorHubReconfigureCB(sensor_hub::SensorHubConfig &config, uint32_t level);
    void sendCommand();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    std::string port_;
    std::string camera_focus_mode_;
    double camera_focusing_params_a0_;
    double camera_focusing_params_a1_;
    double camera_focusing_params_a2_;
    int camera_focus_value_;
    int led_id_;
    double led_duty_;
    int load_cell_samples_;

    SensorHub sensor_hub_;

    boost::shared_ptr<dynamic_reconfigure::Server<sensor_hub::SensorHubConfig>> srv_;
};

} //namespace sensor_hub

#endif //HONGO_SENSOR_HUB_SENSOR_HUB_NODE_H
