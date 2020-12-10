#ifndef SENSOR_HUB_SENSOR_HUB_H
#define SENSOR_HUB_SENSOR_HUB_H

#include <string>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <ros/ros.h>

#include <sensor_hub/protocol.h>

namespace sensor_hub
{

struct termios tio;

class SensorHub
{
public:
    SensorHub();
    ~SensorHub();

    bool openSensorHub(std::string port, int baud);
    void closeSensorHub();
    bool validateCheckSum();
    void readSensorHub();
    void writeSensorHub();

    void inline setPU(bool unlock)
                     {unlock_ = unlock;};
    void inline setMS(std::vector<int> motor_speed)
                     {motor_speed_ = motor_speed;};

    int kFileDiscriptor;
    int error;
    char buff[128], temp[128];
    char *p, *command, *contents, *checksum;

private:
    bool unlock_, prev_unlock_;
    std::vector<int> motor_speed_, prev_motor_speed_;

    Protocol protocol_;

private:
    void sendPU();
    void sendMS();
};

} //namespace sensor_hub

#endif //SENSOR_HUB_SENSOR_HUB_H
