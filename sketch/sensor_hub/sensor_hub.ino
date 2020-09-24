#include "HX711.h"
#include <TimedAction.h>
#include <protocol.h>
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

HX711 scale;
sensor_hub::Protocol protocol_;

char buff[255], temp[255];
char *p, *command, *contents, *checksum;
const String kContents = ":";
const String kChecksum = "~";
const String kFooter = "\n";
int in1 = 7;
int in2 = 8;
int enA = 9;

const double deadband_minus = -30;
const double deadband_plus = 30;

void com_init()
{
    Serial.begin(57600);
    Serial.setTimeout(50);
    Serial.print("Serial Communication Start!");
}

uint8_t calcCheckSum(const String str)
{
    uint8_t checksum = 0x00;

    const char *c = str.c_str();
    for (uint8_t i = 0; i < strlen(c); i++)
        checksum ^= c[i];

    return checksum;
}

bool validateCheckSum()
{
    uint8_t calc_checksum = 0x00;

    if((command == NULL) || (contents == NULL) || (checksum == NULL))
        return false;

    calc_checksum ^= calcCheckSum(command);
    calc_checksum ^= calcCheckSum(kContents);
    calc_checksum ^= calcCheckSum(contents);
    calc_checksum ^= calcCheckSum(kChecksum);

    char cs[2];
    sprintf(cs, "%X", calc_checksum);
    Serial.println(cs);

    if(strcmp(checksum, cs) != 0)
        return false;

    return true;
}

void setup(){
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);
  com_init();
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
}

void serial_read()
{

  String str_buf = Serial.readStringUntil('\n');
  int len = str_buf.length();

  if(0 < len){
    str_buf.toCharArray(buff,255);

    strncpy(temp, buff, strlen(buff));
    temp[strlen(buff)] = '\0';

    if((p = strstr(temp, "WS"))!=NULL)
    {
      Serial.println(temp);
      command = strtok(p, kContents.c_str());
      contents = strtok(NULL, kChecksum.c_str());
      checksum = strtok(NULL, kFooter.c_str());
      Serial.println(command);
      Serial.println(contents);
      Serial.println(checksum);
      if(validateCheckSum())
      {
        double pwm = atoi(contents);

        if(pwm < deadband_minus){
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          analogWrite(enA, abs(pwm));

        } else if(deadband_plus < pwm){
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          analogWrite(enA, abs(pwm));
           
        } else {
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW);

        }
      }
    }
  }

}

void serial_write()
{
    if (scale.is_ready()) {
      long reading = scale.read();
      protocol_.ClearPacket();
      protocol_.AddCommand("LD");
      protocol_.AddContents(String(reading));
      protocol_.AddChecksum();
      protocol_.AddFooter();
      Serial.println(protocol_.getPacket().c_str());
    } else {
      Serial.println("HX711 not found.");
    }

}

TimedAction serial_read_action = TimedAction(100,serial_read);
TimedAction serial_write_action = TimedAction(100,serial_write);

void loop(){
    serial_read_action.check();
    serial_write_action.check();
}
