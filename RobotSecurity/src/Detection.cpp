#include "Detection.h"

/* ------------------ Ultrasonic Sensor ------------------ */

UltrasonicSensor::UltrasonicSensor(uint8_t trig, uint8_t echo)
  : trigPin(trig), echoPin(echo) {}

void UltrasonicSensor::begin() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("Ultrasonic sensor initialized");
}

String UltrasonicSensor::measureDistance() {
  float totalDistance = 0;
  int validReadings = 0;

  for (int i = 0; i < 5; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);

    if (duration > 0) {
      float distance_cm = (duration * 0.0343) / 2.0;
      if (distance_cm >= 2.0 && distance_cm <= 450.0) {
        totalDistance += distance_cm;
        validReadings++;
      }
    }

    delay(10);
  }

  if (validReadings == 0) {
    return "No valid measurement";
  }

  float avg = totalDistance / validReadings;

  if (avg > 15.0) {
    return "LARMA, Distance: " + String(avg, 1) + " cm";
  } else {
    return "ITEM STILL HERE, Distance: " + String(avg, 1) + " cm";
  }
}


/* ------------------ Radar Sensor ------------------ */

RadarSensor::RadarSensor(HardwareSerial &serial, uint8_t rx, uint8_t tx)
  : radarSerial(serial), rxPin(rx), txPin(tx) {}

bool RadarSensor::setup() {
  radarSerial.begin(256000, SERIAL_8N1, rxPin, txPin);
  delay(1000);

  if (radar.begin(radarSerial)) {
    Serial.println("LD2410 radar initialized ok");
    return true;
  } else {
    Serial.println("LD2410 radar not connected");
    return false;
  }
}

bool RadarSensor::detectMovement() {
  uint32_t now = millis();

  if (now - lastRadarCheck < RADAR_CHECK_INTERVAL) {
    return movementActive;
  }

  lastRadarCheck = now;
  radar.read();

  if (!radar.isConnected()) {
    movementActive = false;
    movementReturned = false;
    return false;
  }

  if (radar.movingTargetDetected()) {
    if (movementStart == 0) {
      movementStart = now;
    }

    if (!movementActive && (now - movementStart >= MOVEMENT_THRESHOLD)) {
      movementActive = true;
    }
  } else {
    movementStart = 0;
    movementActive = false;
    movementReturned = false;
  }

  return movementActive;
}
