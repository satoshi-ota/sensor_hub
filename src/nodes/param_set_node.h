#ifndef HONGO_SENSOR_HUB_PARAM_SET_NODE_H
#define HONGO_SENSOR_HUB_PARAM_SET_NODE_H

#include <string.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_hub/SensorHubConfig.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace sensor_hub
{

class ParamSetNode
{
public:
    ParamSetNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~ParamSetNode();

    void SensorHubReconfigureCB(sensor_hub::SensorHubConfig &config, uint32_t level);

    void updateConfiguration();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

};

} //namespace sensor_hub

#endif //HONGO_SENSOR_HUB_PARAM_SET_NODE_H
