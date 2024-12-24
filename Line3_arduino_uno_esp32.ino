#include <WiFi.h>

// IR Sensors
int sensor1 = 13; // Chân GPIO ESP32
int sensor2 = 12;
int sensor3 = 14;
int sensor[3] = {0, 0, 0}; // Initial Values of Sensors

// Motor Variables
int ENA = 25;              // Motor Trái PWM
int motorInput1 = 27;
int motorInput2 = 26;
int motorInput3 = 33;
int motorInput4 = 32;
int ENB = 22;              // Motor Phải PWM

// Ultrasonic Sensor Pins
#define trigPin 5
#define echoPin 18

// Threshold distance to detect obstacle (in cm)
#define OBSTACLE_DISTANCE 10

// PID Constants
float Kp = 25, Ki = 0, Kd = 15;
float error = 0, previous_error = 0, PID_value = 0, I = 0;

// Wi-Fi Configuration (Optional)
const char *ssid = "Your_SSID";
const char *password = "Your_PASSWORD";

void setup() {
  // Setup GPIO
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);

  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  ledcSetup(0, 5000, 8); // Channel 0, 5kHz, 8-bit resolution
  ledcSetup(1, 5000, 8);

  ledcAttachPin(ENA, 0); // Attach ENA to Channel 0
  ledcAttachPin(ENB, 1); // Attach ENB to Channel 1

  Serial.begin(115200);

  // Optional: Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected!");
}

void loop() {
  float distance = measure_distance();
  if (distance < OBSTACLE_DISTANCE && distance > 0) {
    avoid_obstacle();
  } else {
    read_sensor_values();
    handle_movement();
  }
}

void read_sensor_values() {
  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);

  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0) error = 100; // Turn left
  else if (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1) error = 101; // Turn right
  else if (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 0) error = 0; // Go straight
  else error = 102; // Default behavior
}

void handle_movement() {
  if (error == 100) sharpLeftTurn();
  else if (error == 101) sharpRightTurn();
  else calculate_pid();
}

void calculate_pid() {
  float P = error;
  I += error;
  float D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_error = error;

  int left_motor_speed = constrain(100 - PID_value, 0, 255);
  int right_motor_speed = constrain(100 + PID_value, 0, 255);

  ledcWrite(0, left_motor_speed);
  ledcWrite(1, right_motor_speed);

  forward();
}

void forward() {
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}

void sharpLeftTurn() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
  delay(500);
}

void sharpRightTurn() {
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
  delay(500);
}

float measure_distance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return (duration * 0.034) / 2;
}

void avoid_obstacle() {
  sharpRightTurn();
  delay(1000);
  forward();
  delay(500);
}
