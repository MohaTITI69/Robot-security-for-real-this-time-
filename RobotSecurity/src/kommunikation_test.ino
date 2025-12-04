#include <WiFi.h>

// ======= ÄNDRA DETTA TILL DITT NÄTVERK =======
const char* ssid     = "";
const char* password = "";

// IP-adress till datorn där Java-servern körs
const char* serverHost = "192.168.1.132";  // <-- ändra till din PC:s IP
const uint16_t serverPort = 5000;         // Samma som i SimpleSocketServer

// Hur ofta vi vill snacka med servern (ms)
const unsigned long interval = 3000;
unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Startar ESP32-klient...");

  // Koppla upp mot WiFi
  WiFi.begin(ssid, password);
  Serial.print("Ansluter till WiFi: ");
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi anslutet!");
  Serial.print("IP-adress: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Kolla att vi fortfarande är anslutna till WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi tappat, försöker återansluta...");
    WiFi.reconnect();
    delay(1000);
    return;
  }

  unsigned long now = millis();
  if (now - lastSendTime >= interval) {
    lastSendTime = now;
    sendAndReceiveOnce();
  }
}

void sendAndReceiveOnce() {
  Serial.println();
  Serial.println("Försöker koppla upp mot Java-servern...");

  WiFiClient client;

  if (!client.connect(serverHost, serverPort)) {
    Serial.println("❌ Kunde inte ansluta till servern");
    return;
  }

  Serial.println("✅ Ansluten till servern!");

  // ===== Skicka data till servern =====
  String message = "hej från ESP32";
  Serial.print("Skickar: ");
  Serial.println(message);

  // println → skickar text + \r\n, vilket Java:s readLine() gillar
  client.println(message);

  // ===== Ta emot svar från servern =====
  // Java-servern skickar en rad och stänger sedan anslutningen
  client.setTimeout(3000);  // 3 sekunders timeout på svar

  String response = client.readStringUntil('\n');

  if (response.length() > 0) {
    response.trim(); // ta bort \r\n osv
    Serial.print("Svar från servern: ");
    Serial.println(response);
  } else {
    Serial.println("Inget svar (timeout eller tomt svar)");
  }

  // Stäng anslutningen
  client.stop();
  Serial.println("Anslutning stängd.");
}
