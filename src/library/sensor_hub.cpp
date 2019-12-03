#include "sensor_hub/sensor_hub.h"

namespace sensor_hub
{

SensorHub::SensorHub()
{
    error = 0;
    range_ = 0;
    focus_ = 0;
    load_ = 0.0;
}

SensorHub::~SensorHub()
{
    closeSensorHub();
}

bool SensorHub::openSensorHub(std::string port, int baud)
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer, "/dev/%s", port.c_str());
    kFileDiscriptor = open(fileNameBuffer, O_RDWR);
    printf("port:%s, baud rate:%d\n", fileNameBuffer, baud);

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
    int len = read(kFileDiscriptor, buff, sizeof(buff));

    if(0 < len)
    {
        strncpy(temp, buff, strlen(buff));
        temp[strlen(buff)] = '\0';

        if((p = strstr(temp, "DD"))!=NULL)
        {
            command = strtok(p, protocol_.kContents.c_str());
            contents = strtok(NULL, protocol_.kChecksum.c_str());
            checksum = strtok(NULL, protocol_.kFooter.c_str());
            if(validateCheckSum())
            {
                range_ = atof(contents);
            }
        }

        strncpy(temp, buff, strlen(buff));
        temp[strlen(buff)] = '\0';

        if((p = strstr(temp, "CD"))!=NULL)
        {
            command = strtok(p, protocol_.kContents.c_str());
            contents = strtok(NULL, protocol_.kChecksum.c_str());
            checksum = strtok(NULL, protocol_.kFooter.c_str());
            if(validateCheckSum())
            {
                focus_ = atof(contents);
            }
        }

        strncpy(temp, buff, strlen(buff));
        temp[strlen(buff)] = '\0';

        if((p = strstr(temp, "LD"))!=NULL)
        {
            command = strtok(p, protocol_.kContents.c_str());
            contents = strtok(NULL, protocol_.kChecksum.c_str());
            checksum = strtok(NULL, protocol_.kFooter.c_str());
            if(validateCheckSum())
            {
                load_ = atof(contents);
            }
        }
    }
}

void SensorHub::writeSensorHub()
{
    if(camera_focus_mode_ != prev_focus_mode_)
    {
        sendCF();
        prev_focus_mode_ = camera_focus_mode_;
    }

    if(camera_focusing_params_ != prev_focusing_params_)
    {
        sendCP();
        prev_focusing_params_ = camera_focusing_params_;
    }

    if(camera_focus_value_ != prev_focus_value_)
    {
        sendCV();
        prev_focus_value_ = camera_focus_value_;
    }

    if(load_cell_samples_ != prev_cell_samples_)
    {
        sendLS();
        prev_cell_samples_ = load_cell_samples_;
    }

    if(prev_duty_.size() > 0)
    {
        for(int i = 0; i < led_duty_.size(); i++)
            if(led_duty_[i] != prev_duty_[i]) sendDT(i, led_duty_[i]);
        prev_duty_ = led_duty_;
    }
    else
    {
        for(int i = 0; i < led_duty_.size(); i++) sendDT(i, led_duty_[i]);
        prev_duty_ = led_duty_;
    }
};

void SensorHub::sendCF()
{
    protocol_.ClearPacket();
    protocol_.AddCommand("CF");
    protocol_.AddContents(camera_focus_mode_);
    protocol_.AddChecksum();
    protocol_.AddFooter();
    write(kFileDiscriptor, protocol_.getPacket().c_str(),
          strlen(protocol_.getPacket().c_str()));
    // printf("%s\n", protocol_.getPacket().c_str());
}

void SensorHub::sendCP()
{
    std::string contents = "";
    contents.append(std::to_string(camera_focusing_params_[0]));
    contents.append(",");
    contents.append(std::to_string(camera_focusing_params_[1]));
    contents.append(",");
    contents.append(std::to_string(camera_focusing_params_[2]));

    protocol_.ClearPacket();
    protocol_.AddCommand("CP");
    protocol_.AddContents(contents);
    protocol_.AddChecksum();
    protocol_.AddFooter();
    write(kFileDiscriptor, protocol_.getPacket().c_str(),
          strlen(protocol_.getPacket().c_str()));
    // printf("%s\n", protocol_.getPacket().c_str());
}

void SensorHub::sendCV()
{
    protocol_.ClearPacket();
    protocol_.AddCommand("CV");
    protocol_.AddContents(std::to_string(camera_focus_value_));
    protocol_.AddChecksum();
    protocol_.AddFooter();
    write(kFileDiscriptor, protocol_.getPacket().c_str(),
          strlen(protocol_.getPacket().c_str()));
    // printf("%s\n", protocol_.getPacket().c_str());
}

void SensorHub::sendLS()
{
    protocol_.ClearPacket();
    protocol_.AddCommand("LS");
    protocol_.AddContents(std::to_string(load_cell_samples_));
    protocol_.AddChecksum();
    protocol_.AddFooter();
    write(kFileDiscriptor, protocol_.getPacket().c_str(),
          strlen(protocol_.getPacket().c_str()));
    // printf("%s\n", protocol_.getPacket().c_str());
}

void SensorHub::sendDT(int id, float duty)
{
    std::string contents = "";
    contents.append(std::to_string(id+1));
    contents.append(",");
    contents.append(std::to_string(duty));

    protocol_.ClearPacket();
    protocol_.AddCommand("DT");
    protocol_.AddContents(contents);
    protocol_.AddChecksum();
    protocol_.AddFooter();
    write(kFileDiscriptor, protocol_.getPacket().c_str(),
          strlen(protocol_.getPacket().c_str()));
    // printf("%s\n", protocol_.getPacket().c_str());
}

} //namespace sensor_hub
