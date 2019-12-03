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

    void inline setCF(std::string camera_focus_mode)
                     {camera_focus_mode_ = camera_focus_mode;};
    void inline setCP(std::vector<float> camera_focusing_params)
                     {camera_focusing_params_ = camera_focusing_params;};
    void inline setCV(int camera_focus_value)
                     {camera_focus_value_ = camera_focus_value;};
    void inline setDT(std::vector<float> led_duty)
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
    std::string camera_focus_mode_, prev_focus_mode_;
    std::vector<float> camera_focusing_params_, prev_focusing_params_;
    int camera_focus_value_, prev_focus_value_;
    std::vector<float> led_duty_, prev_duty_;
    int load_cell_samples_, prev_cell_samples_;

    Protocol protocol_;

    int range_;
    int focus_;
    double load_;

private:
    void sendCF();
    void sendCP();
    void sendCV();
    void sendLS();
    void sendDT(int id, float duty);
};

} //namespace sensor_hub

#endif //SENSOR_HUB_SENSOR_HUB_H
