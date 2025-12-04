#ifndef NODE_H
#define NODE_H

#include <Arduino.h>

#define MAX_NODES 16

struct Node {
    bool inUse;
    uint8_t mac[6];
    unsigned long lastSeen;
};

#endif
