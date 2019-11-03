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

bool SensorHub::openSensorHub()
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer, "/dev/ttyACM1");
    kFileDiscriptor = open(fileNameBuffer, O_RDWR);

    int baudRate = B57600;

    tio.c_cflag += CREAD;
    tio.c_cflag += CLOCAL;
    tio.c_cflag += CS8;
    tio.c_cflag += 0;
    tio.c_cflag += 0;

    cfsetispeed(&tio, baudRate);
    cfsetospeed(&tio, baudRate);

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
    //read(kFileDiscriptor, buf, sizeof(buf));
    char buff[] = "fasasafsagag\nCD:100~72\nDD:500~71\nLD:100.0~63\nfadsfdsdafas~.:fsadfdas";

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

void SensorHub::sendCF()
{
    protocol_.ClearPacket();
    protocol_.AddCommand("CF");
    protocol_.AddContents(camera_focus_mode_);
    protocol_.AddChecksum();
    protocol_.AddFooter();
    //write(kFileDiscriptor, protocol_.getPacket().c_str(), strlen(protocol_.getPacket().c_str()));
}

void SensorHub::sendCP()
{
    std::string contents = "";
    contents.append(std::to_string(camera_focusing_params_a0_));
    contents.append(",");
    contents.append(std::to_string(camera_focusing_params_a1_));
    contents.append(",");
    contents.append(std::to_string(camera_focusing_params_a2_));

    protocol_.ClearPacket();
    protocol_.AddCommand("CP");
    protocol_.AddContents(contents);
    protocol_.AddChecksum();
    protocol_.AddFooter();
    //write(kFileDiscriptor, protocol_.getPacket().c_str(), strlen(protocol_.getPacket().c_str()));
}

void SensorHub::sendCV()
{
    protocol_.ClearPacket();
    protocol_.AddCommand("CV");
    protocol_.AddContents(std::to_string(camera_focus_value_));
    protocol_.AddChecksum();
    protocol_.AddFooter();
    //write(kFileDiscriptor, protocol_.getPacket().c_str(), strlen(protocol_.getPacket().c_str()));
}

void SensorHub::sendLS()
{
    protocol_.ClearPacket();
    protocol_.AddCommand("LS");
    protocol_.AddContents(std::to_string(load_cell_samples_));
    protocol_.AddChecksum();
    protocol_.AddFooter();
    //write(kFileDiscriptor, protocol_.getPacket().c_str(), strlen(protocol_.getPacket().c_str()));
}

void SensorHub::sendDT()
{
    std::string contents = "";
    contents.append(std::to_string(led_id_));
    contents.append(",");
    contents.append(std::to_string(led_duty_));

    protocol_.ClearPacket();
    protocol_.AddCommand("DT");
    protocol_.AddContents(contents);
    protocol_.AddChecksum();
    protocol_.AddFooter();
    //write(kFileDiscriptor, protocol_.getPacket().c_str(), strlen(protocol_.getPacket().c_str()));
}

} //namespace sensor_hub
