#include "sensor_hub_node.h"

namespace sensor_hub
{

SensorHubNode::SensorHubNode()

{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("port", port_, "ttyUSB0");
    private_nh.param("baud", baud_, 57600);

    if(!sensor_hub_.openSensorHub(port_, baud_)){
        ROS_INFO("Could not open serial port!");
        sensor_hub_.closeSensorHub();
    }

    srv_ = boost::make_shared <dynamic_reconfigure::Server<sensor_hub::SensorHubConfig>>(private_nh);
    dynamic_reconfigure::Server<sensor_hub::SensorHubConfig>::CallbackType cb
        = boost::bind(&SensorHubNode::SensorHubReconfigureCB, this, _1, _2);
    srv_->setCallback(cb);

    sendCommand();

    purge_unit_sub_ = nh.subscribe<std_msgs::Bool>("unlock", 10, &SensorHubNode::commandCB, this);

    write_thread_ = new boost::thread(boost::bind(&SensorHubNode::writeThread, this));

    initialized_ = true;
}

SensorHubNode::~SensorHubNode()
{
    write_thread_->interrupt();
    write_thread_->join();

    delete write_thread_;

    sensor_hub_.closeSensorHub();
}

void SensorHubNode::commandCB(const std_msgs::Bool::ConstPtr& msg)
{
    if(!initialized_)
        return;

    unlock_ = msg->data;
    sendCommand();
}

void SensorHubNode::SensorHubReconfigureCB(
                    sensor_hub::SensorHubConfig &config, uint32_t level)
{
    unlock_ = config.unlock;
    sendCommand();
}

void SensorHubNode::sendCommand()
{
    sensor_hub_.setPU(unlock_);
    sensor_hub_.writeSensorHub();
}

void SensorHubNode::writeThread()
{
    while(initialized_ && ros::ok())
    {
        ros::spinOnce();
    }

    ros::shutdown();
}

} //namespace sensor_hub

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_hub_node");
    sensor_hub::SensorHubNode sensor_hub_node;

    return 0;
}
