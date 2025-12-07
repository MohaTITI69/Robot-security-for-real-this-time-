#include "WifiServerCom.h"
#include <WiFi.h>
#include <WiFiClient.h>

// TODO: byt till ditt riktiga nätverk + server-IP
static const char* WIFI_SSID      = "din mamma";
static const char* WIFI_PASSWORD  = "din mamma";   // undvik å/ä/ö i källkod
static const char* SERVER_IP      = "192.168.137.212";      // datorns IP-adress
static const uint16_t SERVER_PORT = 5000;                 // Java-servern

static WiFiClient serverClient;
static unsigned long lastConnectAttempt = 0;
static const unsigned long RECONNECT_INTERVAL_MS = 5000;

static void ensureWifiConnected() {
    if (WiFi.status() == WL_CONNECTED) return;

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
}

static void ensureServerConnected() {
    if (serverClient.connected()) return;

    unsigned long now = millis();
    if (now - lastConnectAttempt < RECONNECT_INTERVAL_MS) return;
    lastConnectAttempt = now;

    Serial.print("Connecting to server ");
    Serial.print(SERVER_IP);
    Serial.print(":");
    Serial.println(SERVER_PORT);

    if (!serverClient.connect(SERVER_IP, SERVER_PORT)) {
        Serial.println("Server connect failed");
        return;
    }
    Serial.println("Connected to server");
}

void wifiServerSetup() {
    ensureWifiConnected();
    ensureServerConnected();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void wifiServerLoop() {
    // Hålla kopplingen vid liv
    if (WiFi.status() != WL_CONNECTED) {
        ensureWifiConnected();
    }
    if (!serverClient.connected()) {
        ensureServerConnected();
    }
}

bool sendToServer(const String& line) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("sendToServer: no WiFi");
        return false;
    }
    if (!serverClient.connected()) {
        ensureServerConnected();
        if (!serverClient.connected()) {
            Serial.println("sendToServer: server not connected");
            return false;
        }
    }

    String withNewline = line;
    if (!withNewline.endsWith("\n")) withNewline += "\n";

    size_t written = serverClient.print(withNewline);
    return written == withNewline.length();
}
