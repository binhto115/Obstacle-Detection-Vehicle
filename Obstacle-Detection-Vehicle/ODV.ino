#include <Servo.h>
#include <IRremote.h>
#include <stdbool.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
// 13 1 11 10 9 3
const int rs = 13, en = A3, d4 = 11, d5 = 10, d6 = 9, d7 = 3; // en was 12
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define Lpwm_pin 5  //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin 6  //pin of controlling speed---- ENB of motor driver board
#define IR_1 0x00ff6897
#define IR_2 0x00ff9867
#define IR_OK 0x00ff02fd

// UART from stm32l476rgt6
//#define LED_PIN 13
#define RX_PIN A5 // Arduino RX (Receive from stm32 RX)
//#define TX_PIN 0

Servo myservo;
int Echo_Pin = A0;  // ultrasonic module   ECHO to A0
int Trig_Pin = A1;  // ultrasonic module  TRIG to A1
int RECV_PIN = 12;  // IR receiver pin
int pos;
IRrecv irrecv(RECV_PIN);
decode_results results;


int pinLB = 2;  //pin of controlling turning---- IN1 of motor driver board
int pinLF = 4;  //pin of controlling turning---- IN2 of motor driver board
int pinRB = 7;  //pin of controlling turning---- IN3 of motor driver board
int pinRF = 8;  //pin of controlling turning---- IN4 of motor driver board

volatile int D_mix;
volatile int D_mid;
volatile int D_max;

volatile int Front_Distance;
volatile int Left_Distance;
volatile int Right_Distance;
volatile int Right_IR_Value;
volatile int Left_IR_Value;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to check the front distance, where the sensor is facing
float checkdistance() {
  digitalWrite(Trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Pin, LOW);
  float distance = pulseIn(Echo_Pin, HIGH) / 58.00;
  delay(10);
  return distance;
}

// Debug function
void Detect_Left_and_Right__distance() {
  myservo.write(180);
  delay(400);
  Left_Distance = checkdistance();
  delay(600);
  Serial.print("Left_Distance:");
  Serial.println(Left_Distance);
  myservo.write(0);
  delay(400);
  Right_Distance = checkdistance();
  delay(600);
  Serial.print("Right_Distance:");
  Serial.println(Right_Distance);
  myservo.write(90);

  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    irrecv.resume();  // Receive the next value
  }
}

unsigned long Key;
void IR_Control(void) {
  bool SameFunction = true;
  if (irrecv.decode(&results)) {  // check if serial port receives data
    Key = results.value;
    irrecv.resume();

    switch (Key) {
      case IR_1:  // functionality 1
        lcd.clear();
        lcd.print("Function 1");
        delay(1000);
        SameFunction = true;
        while (SameFunction) {
          Function_1();
          if (irrecv.decode(&results)) {  // check for a new IR signal again
            Key = results.value;
            if (Key == IR_OK || Key == IR_2) {
              SameFunction = false;
            }
            irrecv.resume();
          }
        }
        stopp();
        break;
      case IR_2:  // functionality 2
        lcd.clear();
        lcd.print("Function 2");
        delay(1000);
        while (SameFunction) {
          Function_2();
          if (irrecv.decode(&results)) {  // check for a new IR signal again
            Key = results.value;
            if (Key == IR_OK || Key == IR_1) {
              SameFunction = false;
            }
            irrecv.resume();
          }
        }
        stopp();
        break;

      default:
        break;
    }
    irrecv.resume();  // receive the next data
  }
}

// **Car Rotation & Servo Re-Centering Function**
void Turn_Towards_Direction(int servo_angle) {
  myservo.write(servo_angle);
  delay(100);  // Give servo a moment to adjust

  int Angle_to_Turn = abs(90 - servo_angle);  //
  int T90 = 500;                              // Time it takes to turn 90
  int Turn_Time = map(Angle_to_Turn, 0, 90, 0, T90);

  if (servo_angle < 90) {  // Turn Left
    rotate_right(120);
    lcd.clear();
    lcd.print("Turning right..");
    delay(Turn_Time);             // Adjust turn time
  } else if (servo_angle > 90) {  // Turn Right
    rotate_left(120);
    lcd.clear();
    lcd.print("Turning left...");
    delay(Turn_Time);  // Adjust turn time
  }
  stopp();
  myservo.write(90);  // Re-center the servo
  delay(500);         // Allow servo to stabilize
}

// Function to make movements seamless based on distance
void DistanceResponse(int Front_Distance, int best_angle) {
  if (Front_Distance > 0 && Front_Distance <= 10) {
    Turn_Towards_Direction(best_angle);  // rotate the car

    delay(2000); /// Delay to see the turn 
    go_backward(150);

    while (checkdistance() < 20) {
      float tempDistance = checkdistance();
      lcd.clear();
      lcd.print("Moving back...");
      lcd.setCursor(0, 1);
      lcd.print(tempDistance);
      delay(50);
    }
    while ((checkdistance() >= 10) && (checkdistance() <= 30)) {
      lcd.clear();
      lcd.print("Speed: Stop!");
      float tempDistance = checkdistance();
      lcd.setCursor(0, 1);
      lcd.print(tempDistance);
      stopp();
    }
  } else if ((Front_Distance >= 30) && (Front_Distance <= 40)) {
    Turn_Towards_Direction(best_angle);  // rotate the car
    delay(2000); /// Delay to see the turn 
    go_forward(150);
  
    while (checkdistance() >= 20) {
      float tempDistance = checkdistance();
      lcd.clear();
      lcd.print("Moving up...");
      lcd.setCursor(0, 1);
      lcd.print(tempDistance);
      delay(50);
    }
    while ((checkdistance() >= 10) && (checkdistance() <= 30)) {
      float tempDistance = checkdistance();
      lcd.clear();
      lcd.print("Speed: Stop!");
      lcd.setCursor(0, 1);
      lcd.print(tempDistance);
      stopp();
    }
  } else {
    lcd.clear();
    stopp();
  }
}


// Doggy Mode (Move forward/backward depending on the distance of detected object)
void Function_2() {
  bool SameFunction = true;
  myservo.write(90);  // Start the scan with Sensor facing forward
  delay(500);         // give the sensor time to finish

  int best_angle = 90;  // default: best angle to track
  int max_distance = 0;

  // Responsible for scanning left
  // 0 to 180
  while (SameFunction) {
    for (pos = 0; pos <= 180; pos += 2) {
      if (irrecv.decode(&results)) {  // Check for new IR input
        Key = results.value;
        if (Key == IR_OK || Key == IR_1) {  // Stop function if IR_OK or IR_1 is pressed
          SameFunction = false;
          break;
        }
        irrecv.resume();
        }
      myservo.write(pos);  // Servo turns every angle
      delay(15);
      Front_Distance = checkdistance();
      if ((Front_Distance > 0 && Front_Distance <= 10) || (Front_Distance >= 30 && Front_Distance <= 40)) {
        DistanceResponse(Front_Distance, pos);
      }
    }

    // Responsible for scanning right
    for (pos = 180; pos >= 0; pos -= 2) {
      if (irrecv.decode(&results)) {  // Check if new IR signal is received
        Key = results.value;
        if (Key == IR_OK || Key == IR_1) {  // Stop function if IR_OK or IR_1 is pressed
          SameFunction = false;
          break;
        }
        irrecv.resume();
      }

      myservo.write(pos);
      delay(15);
      Front_Distance = checkdistance();  // Read distance at this position
      if ((Front_Distance > 0 && Front_Distance <= 10) || (Front_Distance >= 30 && Front_Distance <= 40)) {
        DistanceResponse(Front_Distance, pos);
      }
    }
  }
  stopp();
  myservo.write(90);
}

// Auto CruiseControl
void Function_1() {
  Front_Distance = checkdistance();  // obtain the value detected by ultrasonic sensor
  if ((Front_Distance <= 10) && (Front_Distance > 0)) {
    lcd.clear();
    lcd.print("Speed: stopped");
    stopp();
    delay(100);
    myservo.write(180);
    delay(500);

    // Check left distance
    Left_Distance = checkdistance();
    delay(100);
    myservo.write(0);
    delay(500);
    Right_Distance = checkdistance();
    delay(100);

    //if distance a1 is greater than a2
    if (Left_Distance > Right_Distance) {
      rotate_left(150);  //turn left
      delay(200);
      myservo.write(90);
      // delay(300);
    } else {              //if the right distance is greater than the left
      rotate_right(150);  // turn right
      delay(200);
      myservo.write(90);
    }

  } else if ((Front_Distance <= 30) && (Front_Distance > 10)) {
    lcd.clear();
    lcd.print("Speed: Low");
    go_forward(50);
  } else if ((Front_Distance <= 50) && (Front_Distance > 30)) {
    lcd.clear();
    lcd.print("Speed: Medium");
    go_forward(100);
  } else if ((Front_Distance <= 100) && (Front_Distance > 50)) {
    lcd.clear();
    lcd.print("Speed: High");
    go_forward(200);
  } else {
    lcd.clear();
    lcd.print("Stop!!!");
    stopp();
  }
}

void Obstacle_Avoidance_Main() {
  IR_Control();
}

// A big loop to keep the car going again and again
void loop() {
  Obstacle_Avoidance_Main();
}

// Initialization function
void setup() {
  myservo.attach(A2);
  // Initializes the interface to the LCD screen, 
  // and specifies the dimensions (width and height) of the display.
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();
  lcd.print("LCD Initialized!");

  D_mix = 10;
  D_mid = 20;
  D_max = 100;

  Front_Distance = 0;
  Left_Distance = 0;
  Right_Distance = 0;

  myservo.write(90);

  // Configure EN pin in Analog port to Digital Port
  pinMode(en, OUTPUT);
  digitalWrite(A3, LOW);  // Ensure it's initialized correctly

  pinMode(Echo_Pin, INPUT);
  pinMode(Trig_Pin, OUTPUT);
  pinMode(pinLB, OUTPUT);     // /pin 2
  pinMode(pinLF, OUTPUT);     // pin 4
  pinMode(pinRB, OUTPUT);     // pin 7
  pinMode(pinRF, OUTPUT);     // pin 8
  pinMode(Lpwm_pin, OUTPUT);  // pin 5 (PWM)
  pinMode(Rpwm_pin, OUTPUT);  // pin 6(PWM)
  irrecv.enableIRIn();        // Start the receiver
  Serial.begin(9600);         // Initializing serial port
}

void go_forward(unsigned char speed_val) {  // speed_val：0~255
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  analogWrite(Lpwm_pin, speed_val);
  analogWrite(Rpwm_pin, speed_val);
}

void go_backward(unsigned char speed_val) {  // speed_val：0~255
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  analogWrite(Lpwm_pin, speed_val);
  analogWrite(Rpwm_pin, speed_val);
}

void rotate_left(unsigned char speed_val) {  // speed_val：0~255
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  analogWrite(Lpwm_pin, speed_val);
  analogWrite(Rpwm_pin, speed_val);
}
void rotate_right(unsigned char speed_val) {  // speed_val：0~255
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  analogWrite(Lpwm_pin, speed_val);
  analogWrite(Rpwm_pin, speed_val);
}

void stopp() {  //stop
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, HIGH);
}