// IR Sensors
int sensor1 = 2;      // Left most sensor
int sensor2 = 3;
int sensor3 = 4;     
//int sensor5 = 5;      // Right most sensor
// Initial Values of Sensors
int sensor[3] = {0, 0, 0};

// Motor Variables
int ENA = 6;              //Motor Trái
int motorInput1 = 7;
int motorInput2 = 8;
int motorInput3 = 9;
int motorInput4 = 10;
int ENB = 11;             //Motor Phải

//Initial Speed of Motor
int initial_motor_speed = 100;

// Output Pins for Led
int ledPin1 = A3;
int ledPin2 = A4;

// PID Constants
float Kp = 25;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

int flag = 0;

void setup()
{
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  //pinMode(sensor5, INPUT);

  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);

  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);

  Serial.begin(9600);                     //setting serial monitor at a default baund rate of 9600
  delay(500);
  Serial.println("Started !!");
  delay(1000);
}

void loop()
{
  read_sensor_values();
  Serial.print(error);
  if (error == 100) {               // Rẽ Trái 90*    Make left turn untill it detects straight path
    //Serial.print("\t");
    //Serial.println("Left");
    //stop_bot();
    //delay(100);
    do {                            // Quay sang trái cho tới khi phát hiện ngay giữa line
      left();
      analogWrite(ENA, 110);        //Left Motor Speed
      analogWrite(ENB, 90);         //Right Motor Speed
      read_sensor_values();
    } while (error == 0);
    //stop_bot();
    //delay(200);
    //read_sensor_values();
  } else if (error == 101) {          // Rẽ Phải 90* Make right turn in case of it detects only right path (it will go into forward direction in case of staright and right "|--")
    // untill it detects straight path.
    //Serial.print("\t");
    //Serial.println("Right");
    //  Quay đầu về bên trái cho tới khi phát hiện line thì dừng lại
      //stop_bot();
      //delay(100);
      do {                          // Quay sang trái cho tới khi phát hiện ngay giữa line
        analogWrite(ENA, 110);        //Left Motor Speed
        analogWrite(ENB, 80);        //Right Motor Speed
        sharpRightTurn();
        read_sensor_values();
      } while (error == 0);
      //stop_bot();
      //delay(200);
      //read_sensor_values();
  } else if (error == 102) {          // Quay đầu về bên trái  Make left turn untill it detects straight path
    //Serial.print("\t");
    //Serial.println("Sharp Left Turn");
      //stop_bot();
      //delay(100); 
      do {                              // Quay sang trái cho tới khi phát hiện ngay giữa line
      read_sensor_values();
      analogWrite(ENA, 110);        //Left Motor Speed
      analogWrite(ENB, 80);             //Right Motor Speed
      sharpLeftTurn();
      } while (error == 0);
      //stop_bot();
      //delay(200);
      //read_sensor_values();
  } else if (error == 103) {          // Make left turn untill it detects straight path or stop if dead end reached.
    stop_bot();
    //delay(100);
    if (flag == 0) {
      analogWrite(ENA, 110);        //Left Motor Speed
      analogWrite(ENB, 80);             //Right Motor Speed
      forward();
      delay(100);
      //stop_bot();
      read_sensor_values();
      if (error == 103) {             /**** Dead End Reached, Stop! ****/
        stop_bot();
        delay(200);
        digitalWrite(ledPin1, HIGH);
        digitalWrite(ledPin2, HIGH);
        flag = 1;
       } else {                        /**** Move Left ****/
        do {                                      // Quay sang trái cho tới khi phát hiện ngay giữa line
          //Serial.print("\t");
          //Serial.println("Left Here");
          analogWrite(ENA, 110);        //Left Motor Speed
          analogWrite(ENB, 80);        //Right Motor Speed
          sharpLeftTurn();
          read_sensor_values();
        } while (error == 0);
        //stop_bot();
        //delay(200);
        //read_sensor_values();
        }
    }
  }else {
    calculate_pid();                  // Tính giá trị PID
    motor_control();                  // Điều chỉnh motor theo giá trị PID mới tính, cho xe chạy thẳng
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
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)) // Dừng lại
    error = 103;
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
  int left_motor_speed = initial_motor_speed - PID_value + 40 ;   // Tính tốc độ của motor trái
  int right_motor_speed = initial_motor_speed + PID_value + 40;  // Tính tốc độ của motor phải

  // Giới hạn tốc độ motor không vượt quá giá trị PWM tối đa
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  // Điều khiển tốc độ động cơ
  analogWrite(ENA, left_motor_speed);
  analogWrite(ENB, right_motor_speed);

  // Điều khiển di chuyển về phía trước hoặc thực hiện hành động theo giá trị error
  if (error == 0) {
    forward();
  } else if (error == 100) {
    left();
  } else if (error == 101) {
    right();
  } else if (error == 102) {
    reverse();
  } else if (error == 103) {
    stop_bot();
  }
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

void sharpLeftTurn() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}

void sharpRightTurn() {
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
