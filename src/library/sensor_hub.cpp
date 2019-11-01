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

bool SensorHub::openSensorHub()
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer, "/dev/ttyACM1");
    kFileDiscriptor = open(fileNameBuffer, O_RDWR);

    int baudRate = B57600;

    tio.c_cflag += CREAD;               // 受信有効
    tio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
    tio.c_cflag += CS8;                 // データビット:8bit
    tio.c_cflag += 0;                   // ストップビット:1bit
    tio.c_cflag += 0;                   // パリティ:None

    cfsetispeed(&tio, baudRate);
    cfsetospeed(&tio, baudRate);

    cfmakeraw(&tio);                    // RAWモード

    tcsetattr(kFileDiscriptor, TCSANOW, &tio);     // デバイスに設定を行う

    ioctl(kFileDiscriptor, TCSETS, &tio);            // ポートの設定を有効にする

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

void SensorHub::readSensorHub()
{
    read(kFileDiscriptor, buf, sizeof(buf));
    //printf("%s\n", buf);
    //char buff[] = "LD:1.234567~52\n";
    char *p;
    p = strstr(buf, "LD");

    char *tilda;
    tilda = strstr(buf, "~");

    char temp[128];
    strncpy(temp, p, abs(tilda - p + 1));
    temp[abs(tilda - p + 1)] = '\0';

    char checknum[2];
    strncpy(checknum, tilda + 1, 2);
    checknum[2] = '\0';

    uint8_t checksum = 0x00;
    for (uint8_t i = 0; i < strlen(temp); i++)
        checksum ^= temp[i];

    printf("%s\n", temp);
    printf("%s\n", checknum);

    char chk[8];
    sprintf(chk, "%X", checksum);

    if(strncmp(chk, checknum, 2) == 0)
    {
        printf("Data Recieve!\n");
        char load[128];
        strncpy(load, p+3, abs(tilda - p));
        load[abs(tilda - p + 1)] = '\0';

        load_ = atof(load);
    }
}

int abs(int num){
   return (num > 0) ? num : -num;
}

void SensorHub::writeSensorHub()
{
    sendCF();
    sendCP();
    sendLS();
}

void SensorHub::sendCF()
{
    ClearPacket();
    //AddHeader();
    AddCommand("CF");
    AddContents(camera_focus_mode_);
    AddChecksum();
    AddFooter();
    //for debugging
    printf("%s\n", packet_.c_str());
    write(kFileDiscriptor, packet_.c_str(), strlen(packet_.c_str()));
}

void SensorHub::sendCP()
{
    std::string contents = "";
    contents.append(std::to_string(camera_focusing_params_a0_));
    contents.append(",");
    contents.append(std::to_string(camera_focusing_params_a1_));
    contents.append(",");
    contents.append(std::to_string(camera_focusing_params_a2_));

    ClearPacket();
    //AddHeader();
    AddCommand("CP");
    AddContents(contents);
    AddChecksum();
    AddFooter();
    //for debugging
    printf("%s\n", packet_.c_str());
    write(kFileDiscriptor, packet_.c_str(), strlen(packet_.c_str()));
}

void SensorHub::sendLS()
{
    ClearPacket();
    //AddHeader();
    AddCommand("LS");
    AddContents(std::to_string(load_cell_samples_));
    AddChecksum();
    AddFooter();
    //for debugging
    printf("%s\n", packet_.c_str());
    write(kFileDiscriptor, packet_.c_str(), strlen(packet_.c_str()));
}

void SensorHub::AddHeader()
{
    packet_header_ = kHeader;
    packet_.append(packet_header_);
}

void SensorHub::AddCommand(const std::string command)
{
    packet_command_ = kCommand;
    packet_command_.append(command);
    packet_.append(packet_command_);
}

void SensorHub::AddContents(const std::string contents)
{
    packet_contents_ = kContents;
    packet_contents_.append(contents);
    packet_.append(packet_contents_);
}

void SensorHub::AddChecksum()
{
    uint8_t checksum = 0x00;

    packet_checksum_ = kChecksum;

    const char* command = packet_command_.c_str();
    const char* contents = packet_contents_.c_str();

    for (uint8_t i = 0; i < strlen(command); i++)
        checksum ^= command[i];

    checksum ^= 0x3a;

    for (uint8_t i = 0; i < strlen(contents); i++)
        checksum ^= contents[i];

    checksum ^= 0x7e;

    char chk[8];
    sprintf(chk,"%X",checksum);

    packet_.append(packet_checksum_);
    packet_.append(chk);
}

void SensorHub::AddFooter()
{
    packet_footer_ = kFooter;
    packet_.append(packet_footer_);
}

void SensorHub::ClearPacket()
{
    packet_ = "";
}

} //namespace sensor_hub
