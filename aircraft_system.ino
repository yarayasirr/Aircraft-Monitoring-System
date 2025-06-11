// Blynk settings
#define BLYNK_TEMPLATE_ID "TMPL5rpvzIwVw"
#define BLYNK_TEMPLATE_NAME "Airplane Monitoring System"
#define BLYNK_AUTH_TOKEN "z9fn8_Tby6pFJxDJzEvmlyU-FyaC4lGX"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// WiFi credentials
char ssid[] = "Ahmed";
char pass[] = "01012066109";

// Pins
#define IR_SENSOR_PIN 33

// Sensor objects
MPU6050 mpu;
Adafruit_BMP280 bmp; // I2C

// Timer
BlynkTimer timer;

// Thresholds
const int ACC_THRESHOLD = 15000;
const int GYRO_THRESHOLD = 15000;
const float TEMP_THRESHOLD = 40.0;     
const float PRESSURE_LOW = 95000.0;
const float PRESSURE_HIGH = 105000.0;

void sendSensorData() {
  // Read MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Read BMP280
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude(1013.25); // Sea level pressure

  // Read IR sensor
  int irValue = digitalRead(IR_SENSOR_PIN);
  int objectDetected = (irValue == LOW) ? 1 : 0;

  // Send to Blynk
  Blynk.virtualWrite(V0, pressure);      // Pressure
  Blynk.virtualWrite(V1, temperature);   // Temp
  Blynk.virtualWrite(V2, altitude);      // Altitude

  Blynk.virtualWrite(V3, ax);            // Accel X
  Blynk.virtualWrite(V4, gx);            // Gyro X
  Blynk.virtualWrite(V5, ay);            // Accel Y
  Blynk.virtualWrite(V6, az);            // Accel Z
  Blynk.virtualWrite(V7, gy);            // Gyro Y
  Blynk.virtualWrite(V8, gz);            // Gyro Z

  Blynk.virtualWrite(V9, objectDetected); // IR Sensor

  // Warnings via virtual LED
  bool warning = false;
  if (abs(ax) > ACC_THRESHOLD || abs(ay) > ACC_THRESHOLD || abs(az) > ACC_THRESHOLD) warning = true;
  if (abs(gx) > GYRO_THRESHOLD || abs(gy) > GYRO_THRESHOLD || abs(gz) > GYRO_THRESHOLD) warning = true;
  if (temperature > TEMP_THRESHOLD) warning = true;
  if (pressure < PRESSURE_LOW || pressure > PRESSURE_HIGH) warning = true;

  Blynk.virtualWrite(V10, warning ? 255 : 0); // Virtual LED ON/OFF
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // ESP32 default I2C pins
  pinMode(IR_SENSOR_PIN, INPUT);

  // Start MPU6050
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("✅ MPU6050 connected");
  } else {
    Serial.println("❌ MPU6050 connection failed");
    while (1);
  }

  // Start BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("❌ BMP280 connection failed at 0x76");
    while (1);
  }

  // Start Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Read sensors every second
  timer.setInterval(1000L, sendSensorData);
}

void loop() {
  Blynk.run();
  timer.run();
}
