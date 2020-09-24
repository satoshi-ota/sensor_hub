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

    void inline setWS(int winch_speed)
                     {winch_speed_ = winch_speed;};
    void inline setLS(int load_cell_samples)
                     {load_cell_samples_ = load_cell_samples;};

    double inline getLoad(){return load_;};

    int kFileDiscriptor;
    int error;
    char buff[128], temp[128];
    char *p, *command, *contents, *checksum;

private:
    int winch_speed_, prev_winch_speed_;
    int load_cell_samples_, prev_load_cell_samples_;

    Protocol protocol_;

    double load_;

private:
    void sendWS();
    void sendLS();
};

} //namespace sensor_hub

#endif //SENSOR_HUB_SENSOR_HUB_H
