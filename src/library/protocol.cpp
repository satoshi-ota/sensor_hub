#include "sensor_hub/protocol.h"

namespace sensor_hub
{

Protocol::Protocol(){ }

Protocol::~Protocol(){ }

uint8_t Protocol::calcCheckSum(const std::string str)
{
    uint8_t checksum = 0x00;

    const char *c = str.c_str();
    for (uint8_t i = 0; i < strlen(c); i++)
        checksum ^= c[i];

    return checksum;
}

uint8_t Protocol::calcCheckSum(const char *c)
{
    uint8_t checksum = 0x00;

    for (uint8_t i = 0; i < strlen(c); i++)
        checksum ^= c[i];

    return checksum;
}

void Protocol::AddCommand(const std::string command)
{
    packet_command_ = command;
    packet_.append(command);
}

void Protocol::AddContents(const std::string contents)
{
    packet_contents_ = kContents;
    packet_contents_.append(contents);
    packet_.append(packet_contents_);
}

void Protocol::AddChecksum()
{
    uint8_t checksum = 0x00;

    checksum ^= calcCheckSum(packet_command_);
    checksum ^= calcCheckSum(packet_contents_);
    checksum ^= calcCheckSum(kChecksum);

    char cs[2];
    sprintf(cs,"%X",checksum);

    packet_checksum_ = kChecksum;
    packet_.append(packet_checksum_);
    packet_.append(cs);
}

void Protocol::AddFooter()
{
    packet_footer_ = kFooter;
    packet_.append(packet_footer_);
}

void Protocol::ClearPacket()
{
    packet_ = "";
}

}
