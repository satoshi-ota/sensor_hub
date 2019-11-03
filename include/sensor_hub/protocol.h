#ifndef SENSOR_HUB_PROTOCOL_H
#define SENSOR_HUB_PROTOCOL_H

#include <string>
#include <string.h>

namespace sensor_hub
{

class Protocol
{
public:
    Protocol();
    ~Protocol();

    void AddCommand(const std::string command);
    void AddContents(const std::string contents);
    void AddChecksum();
    void AddFooter();
    void ClearPacket();

    uint8_t calcCheckSum(const char *c);
    uint8_t calcCheckSum(const std::string str);

    std::string inline getPacket(){return packet_;};

    const std::string kContents = ":";
    const std::string kChecksum = "~";
    const std::string kFooter = "\n";

private:
    std::string packet_;
    std::string packet_command_;
    std::string packet_contents_;
    std::string packet_checksum_;
    std::string packet_footer_;

};

} //namespace sensor_hub

#endif //SENSOR_HUB_PROTOCOL_H
