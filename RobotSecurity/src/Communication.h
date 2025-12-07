#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>
#include "Communication.h"
#include "Node.h"
#include <algorithm>


#define WIFI_CHANNEL 1
#define connectedPin 33
#define msgRecPin 26

static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
const unsigned long TimeUntilOff = 7000;

enum MsgType : uint8_t { HELLO = 1, ROSTER = 2, ANNOUNCE = 3, ACTIVITY = 4 };

#pragma pack(push, 1)
struct MsgHello { uint8_t type; };
struct MsgAnnounce { uint8_t type; uint8_t mac[6]; };
#pragma pack(pop)

extern uint8_t selfMac[6];

bool sameMac(const uint8_t a[6], const uint8_t b[6]);
int findNode(const uint8_t mac[6]);
int countPeers();
void sortNodes();
int insertNodeIfMissing(const uint8_t mac[6]);
void onRecv(const uint8_t* mac, const uint8_t* data, int len);
void onSent(const uint8_t* mac, esp_now_send_status_t status);
void heartbeatSender(void*);
void nodeTimeoutChecker(void*);
void setSelfMacToList();
void send_msg_bcast(const String& msg);
void send_msg_mac(const String& msg, const uint8_t* mac);
void send_bytes_mac(const uint8_t* msg, size_t len, const uint8_t* mac);
void send_bytes_bcast(const uint8_t* msg, size_t len);
String bytesToString(const uint8_t* data, size_t len);
void updateLed();
void sendHello();
void ledBlink();
void setup_communication();


#endif