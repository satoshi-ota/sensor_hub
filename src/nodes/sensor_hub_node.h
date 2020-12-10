#ifndef SENSOR_HUB_SENSOR_HUB_NODE_H
#define SENSOR_HUB_SENSOR_HUB_NODE_H

#include <boost/thread.hpp>
#include <string.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_hub/SensorHubConfig.h>

#include <sensor_hub/sensor_hub.h>

namespace sensor_hub
{

class SensorHubNode
{
public:
    SensorHubNode();
    ~SensorHubNode();

    void sendCommand();
    void writeThread();

    void unlockCommandCB(const std_msgs::Bool::ConstPtr& msg);
    void velCommandCB(const geometry_msgs::Twist::ConstPtr& msg);
    void SensorHubReconfigureCB(sensor_hub::SensorHubConfig &config, uint32_t level);
private:
    ros::Subscriber purge_unit_sub_;
    ros::Subscriber cmd_vel_sub_;

    SensorHub sensor_hub_;

    bool runWriteTh_;
    boost::condition_variable_any write_cond_;
    boost::thread* write_thread_;

    std::string port_;
    int baud_;
    std::vector<int> motor_speed_;
    bool unlock_;
    bool initialized_;
    boost::shared_ptr<dynamic_reconfigure::Server<sensor_hub::SensorHubConfig>> srv_;
};

} //namespace sensor_hub

#endif //SENSOR_HUB_SENSOR_HUB_NODE_H
