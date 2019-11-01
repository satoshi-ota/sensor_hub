#include "sensor_hub_node.h"

namespace sensor_hub
{

SensorHubNode::SensorHubNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    private_nh_.param("hz_", hz_, 20);
    private_nh_.param<std::string>("camera_focus_mode", camera_focus_mode_, "A");
    private_nh_.param("camera_focusing_params_a0", camera_focusing_params_a0_, 0.0);
    private_nh_.param("camera_focusing_params_a1", camera_focusing_params_a1_, 0.0);
    private_nh_.param("camera_focusing_params_a2", camera_focusing_params_a2_, 0.0);
    private_nh_.param("load_cell_samples", load_cell_samples_, 1);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<sensor_hub::SensorHubConfig>>(private_nh);
    dynamic_reconfigure::Server<sensor_hub::SensorHubConfig>::CallbackType cb
        = boost::bind(&SensorHubNode::SensorHubReconfigureCB, this, _1, _2);
    srv_->setCallback(cb);

    load_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>
                                    (hongo_msgs::default_topics::STATUS_LOAD, 0);
    range_pub_ = nh_.advertise<sensor_msgs::Range>
                                    (hongo_msgs::default_topics::STATUS_RANGE, 0);

    sensor_hub_.openSensorHub();
    sendCommand();
}

SensorHubNode::~SensorHubNode()
{
    sensor_hub_.closeSensorHub();
}

void SensorHubNode::SensorHubReconfigureCB(
                    sensor_hub::SensorHubConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request:CF[%s] CP[%f, %f, %f] LS[%d]",
             config.camera_focus_mode.c_str(),
             config.camera_focusing_params_a0,
             config.camera_focusing_params_a1,
             config.camera_focusing_params_a2,
             config.load_cell_samples);

    camera_focus_mode_ = config.camera_focus_mode.c_str();
    camera_focusing_params_a0_ = config.camera_focusing_params_a0;
    camera_focusing_params_a1_ = config.camera_focusing_params_a1;
    camera_focusing_params_a2_ = config.camera_focusing_params_a2;
    load_cell_samples_ = config.load_cell_samples;
    sendCommand();
}

void SensorHubNode::sendCommand()
{
    sensor_hub_.setCF(camera_focus_mode_);
    sensor_hub_.setCPa0(camera_focusing_params_a0_);
    sensor_hub_.setCPa1(camera_focusing_params_a1_);
    sensor_hub_.setCPa2(camera_focusing_params_a2_);
    sensor_hub_.setLS(load_cell_samples_);
    sensor_hub_.writeSensorHub();
}

void SensorHubNode::readSensorData()
{
    sensor_hub_.readSensorHub();

    //range_msg_.header.stamp = ros::Time::now();
    //range_msg_.header.frame_id = "TFmini_frame";
    //range_msg_.min_range = 0.3;
    //range_msg_.max_range = 12.0;
    //range_msg_.range = sensor_hub_.getRange();
    //range_pub_.publish(range_msg_);

    load_msg_.header.stamp = ros::Time::now();
    load_msg_.header.frame_id = "load_cell_frame";
    load_msg_.wrench.force.z = sensor_hub_.getLoad();
    load_pub_.publish(load_msg_);
}

} //namespace sensor_hub

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_hub_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    sensor_hub::SensorHubNode sensor_hub_node(nh, private_nh);

    int hz = sensor_hub_node.getHz();
    ros::Rate loop_rate(hz);

    while (ros::ok())
    {
        sensor_hub_node.readSensorData();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
