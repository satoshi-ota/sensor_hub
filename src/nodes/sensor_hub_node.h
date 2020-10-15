#ifndef SENSOR_HUB_SENSOR_HUB_NODE_H
#define SENSOR_HUB_SENSOR_HUB_NODE_H

#include <boost/thread.hpp>
#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
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

    void readSensorData();
    void sendCommand();

    void writeThread();
    void readThread();

    void commandCB(const std_msgs::Int32::ConstPtr& msg);
    void SensorHubReconfigureCB(sensor_hub::SensorHubConfig &config, uint32_t level);

private:
    ros::Publisher load_pub_;
    ros::Subscriber winch_speed_sub_;

    SensorHub sensor_hub_;

    bool runReadTh_;
    boost::condition_variable_any read_cond_;
    boost::thread* read_thread_;

    bool runWriteTh_;
    boost::condition_variable_any write_cond_;
    boost::thread* write_thread_;

    std::string port_;
    int baud_;

    geometry_msgs::WrenchStamped load_msg_;

    int winch_speed_;
    int load_cell_samples_;
    bool initialized_;

    boost::shared_ptr<dynamic_reconfigure::Server<sensor_hub::SensorHubConfig>> srv_;
};

} //namespace sensor_hub

#endif //SENSOR_HUB_SENSOR_HUB_NODE_H
