// IR Sensors
int sensor1 = 2;      // Left most sensor
int sensor2 = 3;
int sensor3 = 4;     
int sensor[3] = {0, 0, 0};  // Initial Values of Sensors

// Motor Variables
int ENA = 6;              // Motor Trái
int motorInput1 = 7;
int motorInput2 = 8;
int motorInput3 = 9;
int motorInput4 = 10;
int ENB = 11;             // Motor Phải

// Initial Speed of Motor
int initial_motor_speed = 100;

// Output Pins for LED
int ledPin1 = A3;
int ledPin2 = A4;

// PID Constants
float Kp = 25, Ki = 0, Kd = 15;
float error = 0, previous_error = 0, PID_value = 0, I = 0;

// Ultrasonic Sensor Pins
#define trigPin 12
#define echoPin 13

// Threshold distance to detect obstacle (in cm)
#define OBSTACLE_DISTANCE 10

void setup() {
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);

  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);

  Serial.begin(9600);
  delay(500);
  Serial.println("Started!!");
  delay(1000);
}

void loop() {
  float distance = measure_distance();
  if (distance < OBSTACLE_DISTANCE && distance > 0) { // Obstacle detected
    Serial.println("Obstacle detected!");
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

  // Set error based on sensor values
  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0) error = 100; // Turn left
  else if (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1) error = 101; // Turn right
  else if (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 0) error = 0; // Go straight
  else if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0) error = 102; // U-turn
  else if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1) error = 103; // Stop
}

void handle_movement() {
  if (error == 100) {
    sharpLeftTurn();
  } else if (error == 101) {
    sharpRightTurn();
  } else if (error == 102) {
    uTurn();
  } else if (error == 103) {
    stop_bot();
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
  } else {
    calculate_pid();
    motor_control();
  }
}

void calculate_pid() {
  float P = error;
  I += error;
  float D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_error = error;
}

void motor_control() {
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  analogWrite(ENA, left_motor_speed);
  analogWrite(ENB, right_motor_speed);

  forward();
}

void forward() {
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}

void stop_bot() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}

void sharpLeftTurn() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
  delay(500); // Adjust for turn duration
}

void sharpRightTurn() {
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
  delay(500); // Adjust for turn duration
}

void uTurn() {
  sharpLeftTurn();
  delay(1000); // U-turn duration
}

float measure_distance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return (duration * 0.034) / 2; // Distance in cm
}

void avoid_obstacle() {
  stop_bot();
  delay(500);
  sharpRightTurn();
  delay(1000);
  stop_bot();
}
