// IR Sensors
int sensor1 = 2;      // Left most sensor
int sensor2 = 3;
int sensor3 = 4;
// Initial Values of Sensors
int sensor[3] = {0, 0, 0};

// Motor Variables
int ENA = 12;             // Motor Trái
int motorInput1 = 13;
int motorInput2 = 14;
int motorInput3 = 26;
int motorInput4 = 27;
int ENB = 25;             // Motor Phải

// Initial Speed of Motor
int initial_motor_speed = 100;

// Output Pins for Led
int ledPin1 = 32;
int ledPin2 = 33;

// PID Constants
float Kp = 25;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

// Ultrasonic Sensor
#define trigPin 5
#define echoPin 18
float distance = 0.0;

void setup()
{
  // Pin setup
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

  Serial.begin(115200);
  delay(500);
  Serial.println("Started!!");
  delay(1000);
}

void loop()
{
  read_sensor_values();
  read_distance();

  if (distance < 15) {  // Nếu có vật cản trong 15 cm
    stop_bot();
    delay(500);
    reverse();
    delay(500);
    sharpRightTurn();
    delay(500);  // Xoay đầu để tránh vật cản
  } else if (error == 100) {  // Quay trái
    left();
    analogWrite(ENA, 110);
    analogWrite(ENB, 90);
  } else if (error == 101) {  // Quay phải
    right();
    analogWrite(ENA, 90);
    analogWrite(ENB, 110);
  } else if (error == 102) {  // Quay đầu
    sharpLeftTurn();
    analogWrite(ENA, 110);
    analogWrite(ENB, 90);
  } else if (error == 103) {  // Tới đích
    stop_bot();
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    Serial.println("Arrived at destination!");
    delay(1000);
  } else {
    calculate_pid();  // Điều chỉnh PID
    motor_control();
  }
}

void read_sensor_values()
{
  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);

  // Điều kiện cho các hành động của robot
  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0)) // Quay trái
    error = 100;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1)) // Quay phải
    error = 101;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0))  // Đi thẳng
    error = 0;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0)) // Quay đầu (U-turn)
    error = 102;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)) { // Dừng lại (tới đích)
    error = 103;
  }
}

void read_distance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;  // Tính khoảng cách (cm)
}

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  int left_motor_speed = initial_motor_speed - PID_value + 40;   // Tính tốc độ motor trái
  int right_motor_speed = initial_motor_speed + PID_value + 40; // Tính tốc độ motor phải

  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  analogWrite(ENA, left_motor_speed);
  analogWrite(ENB, right_motor_speed);

  if (error == 0) forward();
}

void forward()
{
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}

void reverse()
{
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}

void right()
{
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}

void left()
{
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}

void stop_bot()
{
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}

void sharpLeftTurn()
{
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}

void sharpRightTurn()
{
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
