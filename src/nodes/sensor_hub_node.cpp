#include "sensor_hub_node.h"

namespace sensor_hub
{

SensorHubNode::SensorHubNode()
    :initialized_(false)
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

    load_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("load", 0);
    winch_speed_sub_ = nh.subscribe<std_msgs::Int32>("winch_speed", 10, &SensorHubNode::commandCB, this);

    read_thread_ = new boost::thread(boost::bind(&SensorHubNode::readThread, this));

    initialized_ = true;
}

SensorHubNode::~SensorHubNode()
{
    read_thread_->interrupt();
    read_thread_->join();

    delete read_thread_;

    sensor_hub_.closeSensorHub();
}

void SensorHubNode::readSensorData()
{
    boost::mutex::scoped_lock lock(r_w_mutex_);

    sensor_hub_.readSensorHub();

    load_msg_.header.stamp = ros::Time::now();
    load_msg_.header.frame_id = "load_cell_frame";
    load_msg_.wrench.force.z = sensor_hub_.getLoad();
    load_pub_.publish(load_msg_);
}

void SensorHubNode::readThread()
{
    ros::Rate rate(50);
    while(initialized_ && ros::ok())
    {
        readSensorData();
        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
}

void SensorHubNode::commandCB(const std_msgs::Int32::ConstPtr& msg)
{
    if(!initialized_)
        return;

    winch_speed_ = msg->data;
    sendCommand();
}

void SensorHubNode::SensorHubReconfigureCB(
                    sensor_hub::SensorHubConfig &config, uint32_t level)
{
    winch_speed_ = config.winch_speed;
    load_cell_samples_ = config.load_cell_samples;

    sendCommand();
}

void SensorHubNode::sendCommand()
{
    boost::mutex::scoped_lock lock(r_w_mutex_);

    sensor_hub_.setWS(winch_speed_);
    sensor_hub_.setLS(load_cell_samples_);
    sensor_hub_.writeSensorHub();
}

} //namespace sensor_hub

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_hub_node");
    sensor_hub::SensorHubNode sensor_hub_node;

    return 0;
}
