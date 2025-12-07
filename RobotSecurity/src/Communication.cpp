#include "Communication.h"
#include "Node.h"
#include "aktivitetsplanering.h"

Node nodes[MAX_NODES];
uint8_t selfMac[6];

bool sameMac(const uint8_t a[6], const uint8_t b[6]) {
  for(int i=0; i<6; i++) if(a[i]!=b[i]) return false;
  return true;
}

int findNode(const uint8_t mac[6]) {
  for(int i=0; i<MAX_NODES; i++)
    if(nodes[i].inUse && sameMac(nodes[i].mac,mac)) return i;
  return -1;
}

int countPeers() {
  int c=0;
  for(int i=0; i<MAX_NODES; i++) if(nodes[i].inUse) c++;
  return c;
}

void sortNodes() {
  std::sort(nodes, nodes + MAX_NODES, [](const Node& a, const Node& b) 
  {
    if (!a.inUse || !b.inUse) 
    {
      // "true" först, alltså alla inUse=true kommer före inUse=false
      return a.inUse > b.inUse;
    }
    return memcmp(a.mac, b.mac, 6) < 0;
  });
}


int insertNodeIfMissing(const uint8_t mac[6]) {
  int idx = findNode(mac);
  if(idx >= 0) return idx;

  for(int i=0; i<MAX_NODES; i++) {
    if(!nodes[i].inUse) {
      nodes[i].inUse = true;
      memcpy(nodes[i].mac, mac, 6);
      nodes[i].lastSeen = millis();

      Serial.print("ADDED: ");
      for(int k=0; k<6; k++) {
        if(k) Serial.print(':');
        Serial.printf("%02X", mac[k]);
      }
      Serial.println();

      sortNodes();
      return i;
    }
  }
  Serial.println("List full, cannot add node.");
  return -1;
}

void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
    if(!mac || !data || len <= 0) return;

    const uint8_t* src = mac;   // samma idé, bara annan typ
    int idx = insertNodeIfMissing(src);
    if (idx >= 0) nodes[idx].lastSeen = millis();

    uint8_t type = data[0];

  if (type == HELLO) {
    uint8_t buf[1+1+6];
    buf[0] = ROSTER;
    buf[1] = 1;
    memcpy(&buf[2], selfMac, 6);
    esp_now_send(src, buf, sizeof(buf));
    
    MsgAnnounce a{ANNOUNCE};
    memcpy(a.mac, src, 6);
    esp_now_send(BCAST, (uint8_t*)&a, sizeof(a));
  }
  else if (type == ROSTER && len >= 2) {
    uint8_t count = data[1];
    for (uint8_t i=0; i<count; i++) {
      insertNodeIfMissing(&data[2+i*6]);
    }
  }
  else if (type == ANNOUNCE && len >= 7) {
    insertNodeIfMissing(&data[1]);
  }
  else if (type == ACTIVITY) {
    if (idx >= 0) nodes[idx].lastSeen = millis();
  }

  else {

  //HÄR ANVÄNDER NI MSG SOM STRÄNG TILL VAD SOM HÄLST, GÄRNA SKICKA DEN VIDARE TILL EN ANNAN FUNKTION
  String msg = bytesToString(data, len);
    handleMsg(msg);
  }

}

void handleMsg(String msg) {
    receivedMsg.push_back(msg);
}

void heartbeatSender(void*) {
  MsgHello m{ACTIVITY};
  while(true) {
    esp_now_send(BCAST, (uint8_t*)&m, sizeof(m));
    vTaskDelay(pdMS_TO_TICKS(TimeUntilOff));
   }
}

void nodeTimeoutChecker(void*) {
  while(true) {
    unsigned long now = millis();
    for (int i = 1; i < MAX_NODES; ++i) {
      if (nodes[i].inUse && now - nodes[i].lastSeen > TimeUntilOff) {
        Serial.print("OFFLINE: ");
        for(int k=0;k<6;k++){
          if(k) Serial.print(':');
          Serial.printf("%02X", nodes[i].mac[k]);
        }
        Serial.println();
        nodes[i].inUse = false;
        sortNodes();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void onSent(const uint8_t* mac, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "TX OK" : "TX FAIL");
}

void setSelfMacToList() {
  WiFi.macAddress(selfMac);
  nodes[0].inUse = true;
  memcpy(nodes[0].mac, selfMac, 6);
  nodes[0].lastSeen = millis();

  Serial.print("Self MAC: ");
  for(int k=0; k<6; k++) {
    if(k) Serial.print(':');
    Serial.printf("%02X", selfMac[k]);
  }
  Serial.println();
}

void sendHello() {
  MsgHello m{HELLO};
  esp_err_t r = esp_now_send(BCAST, (uint8_t*)&m, sizeof(m));
  Serial.printf("send HELLO -> %d\n", (int)r);
}

void send_bytes_bcast(const uint8_t* msg, size_t len) {
  Serial.print("Sending BCAST: ");
  for (size_t i = 0; i < len; ++i) {
    Serial.printf("%02X ", msg[i]);
  }
  Serial.println();
  esp_now_send(BCAST, msg, len);
}

void send_bytes_mac(const uint8_t* msg, size_t len, const uint8_t* mac) {
  Serial.print("Sending to MAC ");
  for (int i = 0; i < 6; ++i) {
    if (i) Serial.print(':');
    Serial.printf("%02X", mac[i]);
  }
  Serial.print(": ");
  for (size_t i = 0; i < len; ++i) {
    Serial.printf("%02X ", msg[i]);
  }
  Serial.println();
  esp_now_send(mac, msg, len);
}

void send_msg_bcast(const String& msg) {
  const size_t len = msg.length();
  uint8_t buffer[len];

  memcpy(buffer, msg.c_str(), len);

  send_bytes_bcast(buffer, len);
}

void send_msg_mac(const String& msg, const uint8_t* mac) {
  const size_t len = msg.length();
  uint8_t buffer[len];

  memcpy(buffer, msg.c_str(), len);

  send_bytes_mac(buffer, len, mac);
}

String bytesToString(const uint8_t* data, size_t len) {
    String out = "";
    for (size_t i = 0; i < len; i++) {
        out += (char)data[i];
    }
    return out;
}

void ledBlink() {
  digitalWrite(msgRecPin,HIGH);
  delay(150);
  digitalWrite(msgRecPin,LOW);
}

void updateLed() {
  digitalWrite(connectedPin, countPeers() > 1 ? LOW : HIGH);
}


void setup_communication(){
  for (int i = 0; i < MAX_NODES; i++) nodes[i].inUse = false;

  setSelfMacToList();

  pinMode(connectedPin, OUTPUT);
  pinMode(msgRecPin, OUTPUT);
  digitalWrite(connectedPin, HIGH);

  // WiFi är redan uppkopplat i wifiServerSetup().
  // Viktigt: INTE disconnecta eller byta kanal här.
  WiFi.mode(WIFI_STA);            // ofarligt att kalla igen
  esp_wifi_set_ps(WIFI_PS_NONE);  // power-save av

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESPNOW init failed");
    return;
  }

  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer{};
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, BCAST, 6);
  peer.ifidx = WIFI_IF_STA;
  peer.channel = 0;          // 0 = följ nuvarande WiFi-kanal (AP:ens kanal)
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  xTaskCreatePinnedToCore(heartbeatSender, "Heartbeat", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(nodeTimeoutChecker, "TimeoutCheck", 4096, NULL, 1, NULL, 0);
}


