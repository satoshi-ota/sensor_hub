#ifndef SENSOR_HUB_SENSOR_HUB_H
#define SENSOR_HUB_SENSOR_HUB_H

#include <string>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <sensor_hub/protocol.h>

namespace sensor_hub
{

struct termios tio;

class SensorHub
{
public:
    SensorHub();
    ~SensorHub();

    bool openSensorHub();
    void closeSensorHub();
    bool validateCheckSum();
    void readSensorHub();
    void writeSensorHub(){
        sendCF();
        sendCP();
        sendCV();
        sendLS();
        sendDT();
    };

    void inline setCF(std::string camera_focus_mode)
                     {camera_focus_mode_ = camera_focus_mode;};
    void inline setCPa0(double camera_focusing_params_a0)
                     {camera_focusing_params_a0_ = camera_focusing_params_a0;};
    void inline setCPa1(double camera_focusing_params_a1)
                     {camera_focusing_params_a1_ = camera_focusing_params_a1;};
    void inline setCPa2(double camera_focusing_params_a2)
                     {camera_focusing_params_a2_ = camera_focusing_params_a2;};
    void inline setCV(int camera_focus_value)
                     {camera_focus_value_ = camera_focus_value;};
    void inline setDTid(int led_id)
                     {led_id_ = led_id;};
    void inline setDTduty(double led_duty)
                     {led_duty_ = led_duty;};
    void inline setLS(int load_cell_samples)
                     {load_cell_samples_ = load_cell_samples;};

    int inline getRange(){return range_;};
    int inline getFocus(){return focus_;};
    double inline getLoad(){return load_;};

    int kFileDiscriptor;
    int error;
    char buff[128], temp[128];
    char *p, *command, *contents, *checksum;

private:
    std::string camera_focus_mode_;
    float camera_focusing_params_a0_;
    float camera_focusing_params_a1_;
    float camera_focusing_params_a2_;
    int camera_focus_value_;
    int led_id_;
    float led_duty_;
    int load_cell_samples_;

    Protocol protocol_;

    int range_;
    int focus_;
    double load_;

private:
    void sendCF();
    void sendCP();
    void sendCV();
    void sendLS();
    void sendDT();
};

} //namespace sensor_hub

#endif //SENSOR_HUB_SENSOR_HUB_H
