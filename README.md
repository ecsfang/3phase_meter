# 3phase_meter
Arduinobased 3-phase power meter

Some resources:
* https://github.com/fahrvergnuugen/EmonLib
* https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/ADS1115
* https://github.com/mouse256/monitoring-arduino/tree/master/power-arduino/src
* https://github.com/openenergymonitor/learn/blob/master/view/electricity-monitoring/ctac/emonlib-calibration-theory.md

Note that a file "mySSID.h" should be created. It only contains some IDs and 
passwords and should not be checked in.

<pre>
  char *ssid="The SSID to connect to";
  char *password="The password to use";
  char *mqtt_server="192.xxx.xxx.xxx";
  char *flashpw="Password to use when flashing OTA";
</pre>

Some libraries are needed as well:

* ESP8266 - configuration of the board
* PubSubClient - to be able to use MQTT
* EmonLib - For current and power calculations

Note that the EmonLib should be updated to read from an external ADC.<br>
(TBD: Should update with a link or check-in the changes ...)

Note! PubSublient.h needs to be updated! Increase the following define to 1024:

//#define MQTT_MAX_PACKET_SIZE 128
#define MQTT_MAX_PACKET_SIZE 1024
