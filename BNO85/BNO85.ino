#include <Adafruit_BNO08x.h>
#include <math.h>

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Adafruit BNO08x test!");

  if (!bno08x.begin_I2C(0x4B))// if not working then change the memory address to 4A
   {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }

  Serial.println("BNO08x Found!");

  setReports();
  Serial.println("Reading events");
  delay(100);
}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
}

void loop() {
  delay(10);

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {
    case SH2_GAME_ROTATION_VECTOR:
      float r = sensorValue.un.gameRotationVector.real;
      float i = sensorValue.un.gameRotationVector.i;
      float j = sensorValue.un.gameRotationVector.j;
      float k = sensorValue.un.gameRotationVector.k;

      // Convert quaternion to Yaw (degrees)
      float yaw = atan2(2.0f * (r * k + i * j),
                        1.0f - 2.0f * (j * j + k * k));
      yaw = yaw * (180.0 / M_PI);

      Serial.print("Yaw: ");
      Serial.print(yaw);
      Serial.println(" deg");
      break;
  }
}