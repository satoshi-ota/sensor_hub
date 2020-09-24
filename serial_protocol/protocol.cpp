#include "protocol.h"

namespace sensor_hub
{

Protocol::Protocol(){ }

Protocol::~Protocol(){ }

uint8_t Protocol::calcCheckSum(const String str)
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

void Protocol::AddCommand(const String command)
{
    packet_command_ = command;
    packet_.concat(command);
}

void Protocol::AddContents(const String contents)
{
    packet_contents_ = kContents;
    packet_contents_.concat(contents);
    packet_.concat(packet_contents_);
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
    packet_.concat(packet_checksum_);
    packet_.concat(cs);
}

void Protocol::AddFooter()
{
    packet_footer_ = kFooter;
    packet_.concat(packet_footer_);
}

void Protocol::ClearPacket()
{
    packet_ = "";
}

}
