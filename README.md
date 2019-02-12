# 3phase_meter
Arduinobased 3-phase power meter

Some resources:
* https://github.com/fahrvergnuugen/EmonLib
* https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/ADS1115
* https://github.com/mouse256/monitoring-arduino/tree/master/power-arduino/src

Note that a file "mySSID.h" should be created. It only contains some IDs and 
passwords and should not be checked in.

  char *ssid="The SSID to connect to";
  char *password="The password to use";
  char *mqtt_server="192.xxx.xxx.xxx";
  char *flashpw="Password to use when flashing OTA";

Some libraries are needed as well:

* ESP8266 - configuration of the board
* PubSubClient - to be able to use MQTT
* EmonLib - For current and power calculations

Note that the EmonLib shoudl be updated to read from an external ADC.
(Should update with a link or check-in the changes ...)
