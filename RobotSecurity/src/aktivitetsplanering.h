#ifndef AKTIVITETSPLANERING_H
#define AKTIVITETSPLANERING_H

#include <algorithm>
#include <functional>
#include <vector>
#include <Arduino.h>
#include "esp_system.h"
#include <WiFi.h>
#include "position.hpp"


// ---- struct / enum definitions ----

struct Message 
{
    String messsage;
    int messageId;
    int time;
};


struct Vertex {
    float x;
    float y;
    std::vector<Vertex*> neighbors;
    std::vector<float> distance;
    int room;
    bool checked;
    int id;
};

struct nodeRobot 
{
    bool isMaster;
    String macAddress;
    double battery = 100.0;
    int taskedRoom = 0;
    Vertex* currentVertex;
    Position* pos;
};

enum states {
    INITIATION,
    PATROLLING,
    DETECTING,
    CHARGING,
    ALARMING,
    PLANNING,
    ERROR
};

// ---- GLOBAL VARIABLE DECLARATIONS (extern only) ----

extern int counter;
extern int battery;
extern int msgCount;

extern nodeRobot currentRobot;
extern std::vector<Vertex> nodeList;
extern std::vector<nodeRobot> robotList;
extern bool isMaster;

// ---- function declarations ----

void dealWithMessage(Message msg);
void runningServerChecks(void * parameter);
Message receiveMessage();
void runningReceiveMessages(void * parameter);
void broadcast(Message message);
void sendMessageToServer();
void moveTo(Vertex from, Vertex to);
std::vector<Vertex> getPath(Vertex startPos, Vertex endPos);
Vertex getPosition();
void detectObjects();
void detectObstacles();
void run();
void setUp();
void createMaster();
void charge();
void createsChargeSchedule();
int checkBattery();
String createUpdate();
String plan();
void alarm();
void sortList();
void startNetwork();
void runningUpdateMessages(void * parameter);

#endif // AKTIVITETSPLANERING_H
