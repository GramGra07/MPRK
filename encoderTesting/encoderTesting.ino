#include "FS.h"
#include "SPIFFS.h"
#include <RotaryEncoder.h>
#include <PID_v2.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

WebServer server(80);

// Define the pins
#define PIN_CLK 18
#define PIN_DT 19
#define PIN_SW 21


const int in1 = 8;
const int in2 = 9;
const int ena = 5;

// Create RotaryEncoder object
RotaryEncoder encoder(PIN_DT, PIN_CLK);
const int ticksPerRotation = 12;
int lastSwState;
int encoderValue = 0;
int lastPosition = 0;

double Kp = 1, Ki = 0.1, Kd = 1;
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);

void setup() {
  Serial.begin(115200);
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  encoderValue = readFile("/absolute.txt");
  setupEncoder();
  setupPID();
  setupWifi();
}

void loop() {
  server.handleClient();
  trackEncoder();
  driveMotor();
  serialPIDTuning();
}
void setupMotor() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);
}
void driveMotor() {
  double val = usePID();
  // Serial.println(val);
  if (val > 0) {
    //clockwise
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(ena, val);  // PWM: speed (0–255)
  } else if (val < 0) {

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(ena, val);  // PWM: speed (0–255)
  } else {
    analogWrite(ena, 0);
  }
}
void setupPID() {
  myPID.Start(encoderValue, 0, 0);
}
double usePID() {
  const double input = encoderValue;
  return myPID.Run(input);
}
void setupEncoder() {
  pinMode(PIN_SW, INPUT_PULLUP);
  encoder.setPosition(encoderValue);
  lastPosition = encoderValue;
}

void trackEncoder() {
  int currentSw = digitalRead(PIN_SW);
  encoder.tick();  

  int newPos = encoder.getPosition();
  if (newPos != lastPosition) {
    lastPosition = newPos;
    encoderValue = newPos;
    Serial.print("Position: ");
    Serial.println(newPos);
    Serial.print("Wrapped: ");
    wrap();
    Serial.println(encoderValue);
    writeFile("/absolute.txt", newPos);
  }

  if (lastSwState != currentSw && currentSw == LOW) {
    Serial.println("Button Pressed!");
    encoder.setPosition(0);
    writeFile("/absolute.txt", 0);
  }

  lastSwState = currentSw;
}

void writeFile(const char *path, int message) {
  writeFile(path, String(message).c_str());
}

void writeFile(const char *path, const char *message) {
  File file = SPIFFS.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.print(message);
  file.close();
}
int readFile(const char *path) {
  // Create file with default value if it doesn't exist
  if (!SPIFFS.exists(path)) {
    writeFile(path, 0);
  }

  File file = SPIFFS.open(path, FILE_READ);
  if (!file || file.isDirectory()) {
    Serial.println("Failed to open file for reading");
    return 0;
  }

  String content = file.readStringUntil('\n');
  file.close();

  int value = content.toInt();  // Converts string to int
  return value;
}
void wrap() {
  encoderValue = (encoderValue % ticksPerRotation + ticksPerRotation) % ticksPerRotation;
}

void serialPIDTuning() {
  //kp=2.5
  // ki=0.3
  // kd=1.2

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("kp=")) {
      Kp = input.substring(3).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
    } else if (input.startsWith("ki=")) {
      Ki = input.substring(3).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
    } else if (input.startsWith("kd=")) {
      Kd = input.substring(3).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
    }

    Serial.printf("Kp: %.2f, Ki: %.2f, Kd: %.2f\n", Kp, Ki, Kd);
  }
}
void setupWifi() {
  WiFi.softAP("ESP32-Server", "12345678");
  WiFi.softAPsetHostname("esp32-server");
  if (MDNS.begin("MPRK")) {
    Serial.println("MDNS responder started. Access at http://esp32.local");
  } else {
    Serial.println("Error starting mDNS");
  }


  server.on("/", HTTP_GET, []() {
    String html = "<h1>ESP32 PID Tuning</h1>";
    html += "<form action='/set' method='GET'>";
    html += "Kp: <input name='kp' type='number' step='0.01' value='" + String(Kp) + "'><br>";
    html += "Ki: <input name='ki' type='number' step='0.01' value='" + String(Ki) + "'><br>";
    html += "Kd: <input name='kd' type='number' step='0.01' value='" + String(Kd) + "'><br>";
    html += "<input type='submit' value='Update PID'>";
    html += "</form>";
    server.send(200, "text/html", html);
  });
  server.on("/set", HTTP_GET, []() {
    if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
    if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
    if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();

    myPID.SetTunings(Kp, Ki, Kd);
    myPID.Start(encoderValue, 0, 0);

    // String msg = "<p>PID updated:</p>";
    // msg += "<p>Kp = " + String(Kp) + "</p>";
    // msg += "<p>Ki = " + String(Ki) + "</p>";
    // msg += "<p>Kd = " + String(Kd) + "</p>";
    // msg += "<a href='/'>Back</a>";
    // server.send(200, "text/html", msg);
  });


  server.begin();
}
