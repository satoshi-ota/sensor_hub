#include "sensor_hub/sensor_hub.h"

namespace sensor_hub
{

SensorHub::SensorHub()
{
    error = 0;
}

SensorHub::~SensorHub()
{
    closeSensorHub();
}

bool SensorHub::openSensorHub(std::string port, int baud)
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer, "/dev/%s", port.c_str());
    ROS_INFO("Accsess request...");
    kFileDiscriptor = open(fileNameBuffer, O_RDWR);
    ROS_INFO("port:%s, baud rate:%d\n", fileNameBuffer, baud);

    speed_t speed;
    switch (baud)
    {
    case 4800:     speed = B4800;
                   break;
    case 9600:     speed = B9600;
                   break;
    case 19200:    speed = B19200;
                   break;
    case 38400:    speed = B38400;
                   break;
    case 57600:    speed = B57600;
                   break;
    case 115200:   speed = B115200;
                   break;
    }

    //int baudRate = speed;

    tio.c_cflag += CREAD;
    tio.c_cflag += CLOCAL;
    tio.c_cflag += CS8;
    tio.c_cflag += 0;
    tio.c_cflag += 0;

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    cfmakeraw(&tio);

    tcsetattr(kFileDiscriptor, TCSANOW, &tio);

    ioctl(kFileDiscriptor, TCSETS, &tio);

    if(kFileDiscriptor < 0)
    {
        error = errno;
        return false;
    }

    return true;
}

void SensorHub::closeSensorHub()
{
    if(kFileDiscriptor > 0)
    {
        close(kFileDiscriptor);
        kFileDiscriptor = -1;

        ROS_INFO("sensor_hub shutdown.");
    }
}

bool SensorHub::validateCheckSum()
{
    uint8_t calc_checksum = 0x00;

    if((command == NULL) || (contents == NULL) || (checksum == NULL))
        return false;

    calc_checksum ^= protocol_.calcCheckSum(command);
    calc_checksum ^= protocol_.calcCheckSum(protocol_.kContents);
    calc_checksum ^= protocol_.calcCheckSum(contents);
    calc_checksum ^= protocol_.calcCheckSum(protocol_.kChecksum);

    char cs[2];
    sprintf(cs, "%X", calc_checksum);

    if(strcmp(checksum, cs) != 0)
        return false;

    return true;
}

void SensorHub::readSensorHub()
{
    // int len = read(kFileDiscriptor, buff, sizeof(buff));
    //
    // if(0 < len)
    // {
    //     strncpy(temp, buff, strlen(buff));
    //     temp[strlen(buff)] = '\0';
    //
    //     if((p = strstr(temp, "LD"))!=NULL)
    //     {
    //         command = strtok(p, protocol_.kContents.c_str());
    //         contents = strtok(NULL, protocol_.kChecksum.c_str());
    //         checksum = strtok(NULL, protocol_.kFooter.c_str());
    //         if(validateCheckSum())
    //         {
    //             load_ = atof(contents);
    //         }
    //     }
    // }
}

void SensorHub::writeSensorHub()
{
    if(unlock_ != prev_unlock_)
    {
        sendPU();
        prev_unlock_ = unlock_;
    }

    if(motor_speed_ != prev_motor_speed_)
    {
        sendMS();
        prev_motor_speed_ = motor_speed_;
    }
}

void SensorHub::sendPU()
{
    protocol_.ClearPacket();
    protocol_.AddCommand("PU");
    protocol_.AddContents(std::to_string(unlock_));
    protocol_.AddChecksum();
    protocol_.AddFooter();
    write(kFileDiscriptor, protocol_.getPacket().c_str(),
          strlen(protocol_.getPacket().c_str()));
    ROS_INFO("%s\n", protocol_.getPacket().c_str());
}

void SensorHub::sendMS()
{
    std::string contents = "";
    contents.append(std::to_string(motor_speed_[0]));
    contents.append(",");
    contents.append(std::to_string(motor_speed_[1]));

    protocol_.ClearPacket();
    protocol_.AddCommand("MS");
    protocol_.AddContents(contents);
    protocol_.AddChecksum();
    protocol_.AddFooter();
    write(kFileDiscriptor, protocol_.getPacket().c_str(),
          strlen(protocol_.getPacket().c_str()));
    ROS_INFO("%s\n", protocol_.getPacket().c_str());
}

} //namespace sensor_hub
