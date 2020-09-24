#ifndef SENSOR_HUB_PROTOCOL_H
#define SENSOR_HUB_PROTOCOL_H

#include <stdio.h>
#include <WString.h>

namespace sensor_hub
{

class Protocol
{
public:
    Protocol();
    ~Protocol();

    void AddCommand(const String command);
    void AddContents(const String contents);
    void AddChecksum();
    void AddFooter();
    void ClearPacket();

    uint8_t calcCheckSum(const char *c);
    uint8_t calcCheckSum(const String str);

    String inline getPacket(){return packet_;};

    const String kContents = ":";
    const String kChecksum = "~";
    const String kFooter = "\n";

private:
    String packet_;
    String packet_command_;
    String packet_contents_;
    String packet_checksum_;
    String packet_footer_;

};

} //namespace sensor_hub

#endif //SENSOR_HUB_PROTOCOL_H
