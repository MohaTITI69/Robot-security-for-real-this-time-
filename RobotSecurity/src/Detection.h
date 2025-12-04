#ifndef DETECTION_S
#define DETECTION_S

#include <Arduino.h>
#include <ld2410.h>

/* ------------------ Ultrasonic Sensor ------------------ */
class UltrasonicSensor {
  public:
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin);
    void begin();
    String measureDistance();

  private:
    uint8_t trigPin;
    uint8_t echoPin;
};

/* ------------------ Radar Sensor ------------------ */
class RadarSensor {
  public:
    RadarSensor(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin);

    bool setup();
    bool detectMovement();

  private:
    HardwareSerial &radarSerial;
    uint8_t rxPin;
    uint8_t txPin;
    ld2410 radar;

    uint32_t lastRadarCheck = 0;
    uint32_t movementStart = 0;
    bool movementActive = false;
    bool movementReturned = false;

    const uint32_t RADAR_CHECK_INTERVAL = 200;
    const uint32_t MOVEMENT_THRESHOLD = 1000;
};


class SensorsManager {
  public:
    SensorsManager(UltrasonicSensor &ultra, RadarSensor &radar)
      : ultra(ultra), radar(radar) {}

    void begin() {
      ultra.begin();
      radar.setup();
    }

  private:
    UltrasonicSensor &ultra;
    RadarSensor &radar;
};


#endif
