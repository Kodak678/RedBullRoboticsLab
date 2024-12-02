#include <Servo.h>

// Motor Pins
#define speedPinR 9   // Right motor PWM
#define RightDirectPin1  12    // Right motor direction
#define RightDirectPin2  11
#define speedPinL 6   // Left motor PWM
#define LeftDirectPin1  7     // Left motor direction
#define LeftDirectPin2  8

// Ultrasonic Sensor Pins
#define Trig_PIN 4   // Trig pin for ultrasonic sensor
#define Echo_PIN 5   // Echo pin for ultrasonic sensor

// Other Pins
#define BUZZ_PIN 13   // Buzzer pin
#define SERVO_PIN 9   // Servo motor pin

// Constants
#define SPEED 100      // Speed of motors
#define DIST_LIMIT 30  // Distance limit for obstacles in cm
#define TURN_TIME 300  // Time for turning

Servo head;  // Servo motor for ultrasonic sensor

// Function to move forward
void moveForward() {
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
  
  analogWrite(speedPinR, SPEED);  // Set speed for right motor
  analogWrite(speedPinL, SPEED);  // Set speed for left motor

  Serial.println("Moving Forward");
  Serial.print("Right Motor Speed: "); Serial.println(SPEED);
  Serial.print("Left Motor Speed: "); Serial.println(SPEED);
}

// Function to turn left
void turnLeft() {
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
  
  analogWrite(speedPinR, SPEED);  // Set speed for right motor
  analogWrite(speedPinL, SPEED);  // Set speed for left motor
  
  Serial.println("Turning Left");
}

// Function to turn right
void turnRight() {
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
  
  analogWrite(speedPinR, SPEED);  // Set speed for right motor
  analogWrite(speedPinL, SPEED);  // Set speed for left motor
  
  Serial.println("Turning Right");
}

// Function to stop the car
void stopCar() {
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, LOW);
  
  analogWrite(speedPinR, 0);  // Stop right motor
  analogWrite(speedPinL, 0);  // Stop left motor
  
  Serial.println("Stopped");
}

// Function to trigger the buzzer
void buzzerAlert() {
  digitalWrite(BUZZ_PIN, LOW);
  delay(100);
  digitalWrite(BUZZ_PIN, HIGH);
}

// Function to measure distance using ultrasonic sensor
int getDistance() {
  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_PIN, LOW);
  long duration = pulseIn(Echo_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

// Function to avoid obstacles
void avoidObstacle() {
  int distance = getDistance();
  
  if (distance < DIST_LIMIT) {
    stopCar();
    buzzerAlert();
    
    // Check sides and turn accordingly
    head.write(160);  // Turn servo to left
    delay(500);
    int leftDistance = getDistance();
    
    head.write(20);  // Turn servo to right
    delay(500);
    int rightDistance = getDistance();
    
    head.write(90);  // Reset servo to center
    
    // Decide which way to turn
    if (leftDistance > rightDistance) {
      turnLeft();
    } else {
      turnRight();
    }
    delay(TURN_TIME);  // Turning time
    stopCar();
  } else {
    moveForward();
  }
}

void setup() {
  // Setup motor pins
  pinMode(RightDirectPin1, OUTPUT);
  pinMode(RightDirectPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);
  
  // Setup ultrasonic sensor pins
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);
  
  // Setup buzzer
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);  // Buzzer off
  
  // Setup servo
  head.attach(SERVO_PIN);
  head.write(90);  // Start with servo facing forward
  
  Serial.begin(9600);  // Start serial communication for debugging
}

void loop() {
  avoidObstacle();  // Run obstacle avoidance function
}
