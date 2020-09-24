#ifndef SENSOR_HUB_SENSOR_HUB_NODE_H
#define SENSOR_HUB_SENSOR_HUB_NODE_H

#include <thread>
#include <string.h>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_hub/SensorHubConfig.h>

#include <sensor_hub/sensor_hub.h>

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

    ros::Publisher load_pub_;

    SensorHub sensor_hub_;

    std::string port_;
    int baud_;

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
    int baud_;
    int winch_speed_;
    int load_cell_samples_;

    SensorHub sensor_hub_;

    boost::shared_ptr<dynamic_reconfigure::Server<sensor_hub::SensorHubConfig>> srv_;
};

} //namespace sensor_hub

#endif //SENSOR_HUB_SENSOR_HUB_NODE_H
