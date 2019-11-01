#ifndef HONGO_SENSOR_HUB_SENSOR_HUB_H
#define HONGO_SENSOR_HUB_SENSOR_HUB_H

#include <string>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <locale.h>

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

    void readSensorHub();
    void writeSensorHub();

    void inline setCF(std::string camera_focus_mode)
                     {camera_focus_mode_ = camera_focus_mode;};
    void inline setCPa0(double camera_focusing_params_a0)
                     {camera_focusing_params_a0_ = camera_focusing_params_a0;};
    void inline setCPa1(double camera_focusing_params_a1)
                     {camera_focusing_params_a1_ = camera_focusing_params_a1;};
    void inline setCPa2(double camera_focusing_params_a2)
                     {camera_focusing_params_a2_ = camera_focusing_params_a2;};
    void inline setLS(int load_cell_samples)
                     {load_cell_samples_ = load_cell_samples;};

    double inline getLoad(){return load_;};
    double inline getRange(){return range_;};

    int kFileDiscriptor;
    int error;
    char buf[64];

private:
    void AddHeader();
    void AddCommand(const std::string command);
    void AddContents(const std::string contents);
    void AddChecksum();
    void AddFooter();
    void ClearPacket();

    void sendCF();
    void sendCP();
    void sendLS();

    const std::string kHeader = "$";
    const std::string kCommand = "#";
    const std::string kContents = ":";
    const std::string kChecksum = "~";
    const std::string kFooter = "\n";

    double load_;
    double range_;

    std::string camera_focus_mode_;
    double camera_focusing_params_a0_;
    double camera_focusing_params_a1_;
    double camera_focusing_params_a2_;
    int load_cell_samples_;

    std::string packet_;
    std::string packet_header_;
    std::string packet_command_;
    std::string packet_contents_;
    std::string packet_checksum_;
    std::string packet_footer_;
};

} //namespace sensor_hub

#endif //HONGO_SENSOR_HUB_SENSOR_HUB_H
