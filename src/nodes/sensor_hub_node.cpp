#include "sensor_hub_node.h"

namespace sensor_hub
{

SensorReadNode::SensorReadNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    private_nh_.param<std::string>("port", port_, "ttyACM1");
    private_nh_.param("baud", baud_, 57600);

    load_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>
                                    ("/sensor_hub/load", 0);

    sensor_hub_.openSensorHub(port_, baud_);
}

SensorReadNode::~SensorReadNode()
{
    sensor_hub_.closeSensorHub();
}

void SensorReadNode::readSensorData()
{
    sensor_hub_.readSensorHub();

    load_msg_.header.stamp = ros::Time::now();
    load_msg_.header.frame_id = "load_cell_frame";
    load_msg_.wrench.force.z = sensor_hub_.getLoad();
    load_pub_.publish(load_msg_);
}

SensorWriteNode::SensorWriteNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh),
     initialized_(false)
{
    private_nh_.param<std::string>("port", port_, "ttyUSB0");
    private_nh_.param("baud", baud_, 57600);

    if(!sensor_hub_.openSensorHub(port_, baud_)){
        ROS_INFO("Could not open serial port!");
    }

    srv_ = boost::make_shared <dynamic_reconfigure::Server<sensor_hub::SensorHubConfig>>(private_nh);
    dynamic_reconfigure::Server<sensor_hub::SensorHubConfig>::CallbackType cb
        = boost::bind(&SensorWriteNode::SensorHubReconfigureCB, this, _1, _2);
    srv_->setCallback(cb);

    sendCommand();

    winch_speed_sub_ = nh_.subscribe<std_msgs::Int32>("winch_speed", 10, &SensorWriteNode::commandCB, this);

    initialized_ = true;
}

SensorWriteNode::~SensorWriteNode()
{
    sensor_hub_.closeSensorHub();
}

void SensorWriteNode::commandCB(const std_msgs::Int32::ConstPtr& msg)
{
    if(!initialized_)
        return;

    winch_speed_ = msg->data;
    sendCommand();
}

void SensorWriteNode::SensorHubReconfigureCB(
                    sensor_hub::SensorHubConfig &config, uint32_t level)
{
    winch_speed_ = config.winch_speed;
    load_cell_samples_ = config.load_cell_samples;

    sendCommand();
}

void SensorWriteNode::sendCommand()
{
    sensor_hub_.setWS(winch_speed_);
    sensor_hub_.setLS(load_cell_samples_);
    sensor_hub_.writeSensorHub();
}

} //namespace sensor_hub

void readThread()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    sensor_hub::SensorReadNode sensor_read_node(nh, private_nh);

    while(ros::ok())
    {
        sensor_read_node.readSensorData();
        ros::spinOnce();
    }

    ros::shutdown();
}

void writeThread()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    sensor_hub::SensorWriteNode sensor_write_node(nh, private_nh);

    while(ros::ok())
    {
        ros::spin();
    }

    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_hub_node");

    std::thread th_read(readThread);
    std::thread th_write(writeThread);

    th_read.join();
    th_write.join();

    return 0;
}
