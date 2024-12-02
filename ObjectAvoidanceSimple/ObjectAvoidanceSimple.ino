//obstacle avoidance Code 2
// The labels at the board are opposite.
// So the pins assigned are swapped.

// this constant won't change. It's the pin number of the sensor's output:
//Pin definitions for Ultrasonic obstacle sensor
const int pingPin = 4;
const int echoPin = 5;

long duration;
float distance;
long duration_current;


//Pin definitions for Left motor
int enable_motor_left=6;
int In_left_a=7;
int In_left_b=8;


//Pin definitions for Right motor
int enable_motor_right=9;
int In_right_a=12;
int In_right_b=11;
int PWM = 80;


void setup() 
{
  pinMode(pingPin, OUTPUT); // intitiate PING transmit
  pinMode(echoPin, INPUT); // initiate PING echo receive

  
  
  //Configuring enable and control pins of Left motor
  pinMode(enable_motor_left, OUTPUT);
  pinMode(In_left_a, OUTPUT);
  pinMode(In_left_b, OUTPUT);
  
  //Configuring enable and control pins of Right motor
  pinMode(enable_motor_right, OUTPUT);
  pinMode(In_right_a, OUTPUT);
  pinMode(In_right_b, OUTPUT);

  Serial.begin(9600);  
}

void loop() 
{

  distance = distance_cm(); //calling the distance calculation subroutine
  //Serial.println(distance);


//Defining the operations to be performed based on the distance calculated from the obstacle


  if (distance<10)              //If the distance to obstacle is less than 10 cm,
  {                             //Move back
    right_motor_backward(100);
    left_motor_backward(100); 
    Serial.println("Moving away from you!!!");
  }
  
  else if ((distance<20) && (distance>10))  //If the distance is between 10 cm and 20 cm,
  {                                         //Stop and mantain the distance
    right_motor_forward(0);
    left_motor_forward(0); 
    Serial.println("Waiting for you to move. :D ");
  }

  else                               //If not obstacle is detected, Keep moving forward
  {
    right_motor_forward(100);
    left_motor_forward(100);
    Serial.println("Following you"); 
  }
  delay(100);
  
}

void right_motor_forward(int pwm)
{
  analogWrite(enable_motor_right,pwm);  // Switching the enabling pin of left motor high
  digitalWrite(In_right_a,HIGH);            // setting one directional pin of motor to high
  digitalWrite(In_right_b,LOW);
  
}


void right_motor_backward(int pwm)
{
  analogWrite(enable_motor_right,pwm);  // Switching the enabling pin of left motor high
  digitalWrite(In_right_a,LOW);            // setting one directional pin of motor to high
  digitalWrite(In_right_b,HIGH);
  
}


void left_motor_forward(int pwm)
{
  analogWrite(enable_motor_left,pwm);  // Switching the enabling pin of left motor high
  digitalWrite(In_left_a,HIGH);             // setting one directional pin of motor to high
  digitalWrite(In_left_b,LOW);            // setting other pin of left motor to low
}


void left_motor_backward(int pwm)
{
  analogWrite(enable_motor_left,pwm);  // Switching the enabling pin of left motor high
  digitalWrite(In_left_a,LOW);             // setting one directional pin of motor to high
  digitalWrite(In_left_b,HIGH);            // setting other pin of left motor to low
}

long duration_rout()                    //Sub-routine to calculate distance from the obstacle
{
   digitalWrite(pingPin, LOW);
  delayMicroseconds(2);  
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return duration;
}

float distance_cm()                     //Sub-routine to convert calculated distance to cm
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  duration_current = duration_rout();
  return duration_current / 2.9 / 20;
}

