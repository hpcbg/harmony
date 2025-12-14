#ifndef __CONFIG_H__
#define __CONFIG_H__


// WiFi settings
const char* ssid = "<SSID>";
const char* password = "<password>";

// Fiware settings
const char* fiware_server = "http://192.168.1.105:1026";
const char* fiware_service = "openiot";
const char* fiware_servicepath = "/";
const char* device_id = "M5Stick:001";
const long interval = 1000;  // sending interval (ms)


#endif