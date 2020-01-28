#include "param_set_node.h"

namespace sensor_hub
{

ParamSetNode::ParamSetNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh){}

ParamSetNode::~ParamSetNode(){}

void ParamSetNode::updateConfiguration()
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter duty_param;
    dynamic_reconfigure::Config conf;

    duty_param.name = "led_duty1";
    duty_param.value = 0.5;
    conf.doubles.push_back(duty_param);

    srv_req.config = conf;

    if (ros::service::call("/sensor_hub_node/set_parameters", srv_req, srv_resp)) {
        ROS_INFO("call to set sensor_hub parameters succeeded");
    } else {
        ROS_INFO("call to set sensor_hub parameters failed");
    }
}

} //namespace sensor_hub

int main(int argc, char** argv)
{
    ros::init(argc, argv, "param_set_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    sensor_hub::ParamSetNode param_set_node(nh, private_nh);

    while(ros::ok())
    {
        param_set_node.updateConfiguration();
        ros::spinOnce();
        ros::Duration(5.0).sleep();
    }

    return 0;
}
