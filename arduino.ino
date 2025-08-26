#include "Arduino.h"
#include "Enes100.h"




// Motor control pins
const int enA = 11;
const int in1 = 52;  //motor 1
const int in2 = 53;


const int enB = 44;
const int in3 = 31;  //motor 2
const int in4 = 32;


const int enC = 45;
const int in5 = 41;  //motor 3
const int in6 = 42;


const int enD = 2;
const int in7 = 3;   //motor 4
const int in8 = 4;  




// To account for warping in the actual chassis, adjust as needed later
// Needs to be 0 < corr <= 1
const float corr1 = 1;
const float corr2 = 0.94;
const float corr3 = 0.9;
const float corr4 = 0.94;


const int angularSpeed = 1273; // this*rad = angle you turn at @ 255 power
double testSpeed = 0.9;


const int motorPin1 = 8; 
const int motorPin2 = 9;


// Servo control pin
const int servoPin = 6;
Servo servo;


// Hall effect sensor pin
const int hallSensorPin = 2; 


const int trigPin = 10;
const int echoPin = 11;
const int distanceThreshold = 15; // ultrasonic sensor pin






void setup() {
  Serial.begin(9600);
  // Enes100.begin("-Data Guppies", DATA, 495, 1116, 12, 13);
 
  pinMode(enA, OUTPUT);  
  pinMode(in1, OUTPUT); //configure motor 1
  pinMode(in2, OUTPUT);




  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT); //configure motor 2
  pinMode(in4, OUTPUT);




  pinMode(enC, OUTPUT);
  pinMode(in5, OUTPUT); //configure motor 3
  pinMode(in6, OUTPUT);




  pinMode(enD, OUTPUT);
  pinMode(in7, OUTPUT); //configure motor 4
  pinMode(in8, OUTPUT);


  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT); //configure ultrasonic sensor
  Serial.begin(9600);


  pinMode(motorPin1, OUTPUT); //configure claw motor and hall sensor
  pinMode(motorPin2, OUTPUT);
  pinMode(hallSensorPin, INPUT);


  servo.attach(servoPin); //configure servo








  // Stop motor
  Serial.println("Motors initialized");
  float x = Enes100.getX();
  float y = Enes100.getY();
  float theta = Enes100.getTheta();
}


// Power top-left motor
void pwrTL(int speed) {
  speed = -1 * speed * corr1;
  if (speed > 0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, speed);
  }
  else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, -speed);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  }
}
void pwrTR(int speed) {
  speed = speed * corr2;
  if (speed > 0){
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, speed);
  }
  else if (speed < 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, -speed);
  }
  else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, 0);
  }
}
void pwrBL(int speed) {
  speed = -1 * speed * corr3;
  if (speed > 0){
    digitalWrite(in5, HIGH);
    digitalWrite(in6, LOW);
    analogWrite(enC, speed);
  }
  else if (speed < 0) {
    digitalWrite(in5, LOW);
    digitalWrite(in6, HIGH);
    analogWrite(enC, -speed);
  }
  else {
    digitalWrite(in5, LOW);
    digitalWrite(in6, LOW);
    analogWrite(enC, 0);
  }
}
void pwrBR(int speed) {
  speed = speed  * corr4;
  if (speed > 0){
    digitalWrite(in7, HIGH);
    digitalWrite(in8, LOW);
    analogWrite(enD, speed);
  }
  else if (speed < 0) {
    digitalWrite(in7, LOW);
    digitalWrite(in8, HIGH);
    analogWrite(enD, -speed);
  }
  else {
    digitalWrite(in7, LOW);
    digitalWrite(in8, LOW);
    analogWrite(enD, 0);
  }
}
void pwrMotors(int mot1, int mot2, int mot3, int mot4){
  pwrTL(mot1);
  pwrTR(mot2);
  pwrBL(mot3);
  pwrBR(mot4);
}


// OTV class
class OTV{
  public:
  double x, y, theta;
  bool puck_is_magnetic;


  bool walls[3];
  int case_num = 0;
 
  // Keeps track of/updates coords
  OTV(double a, double b, double c){
      x = a;
      y = b;
      theta = c;
  }
  double getX(){
      //x = Enes100.getX()
      return x;
  }
  double getY(){
      //y = Enes100.getY()
      return y;
  }
  double getTheta(){
      //theta = Enes100.getTheta()
      return theta;
  }
  // Updates based on enes.get
  void update(){
      getX();
      getY();
      getTheta();
  }


  // Manually update values
  void force_update(double a, double b, double c){
      x = x + a;
      y = y + b;
      theta = theta + c;
  }
 
  // Move to x = a, y = b, update coordinates after.
  void moveTo(double a, double b){
    vector_to(x, y, theta, a, b);
    update();
  }
 
  // Rotates to an angle
  void rotateTo(double a){
    simple_turn(a);
    update();
  }
  void stopClawMotor() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}


void lowerClawToGround() {
  digitalWrite(motorPin1, LOW);   // Spin motor in one direction to lower
  digitalWrite(motorPin2, HIGH);
  delay(1500);  // Adjust this delay based on actual time it takes to reach the bottom
  stopMotor();
}


void closeClaw() {
  servo.write(0);
  delay(1000);
}


void openClaw() {
  servo.write(90);
  delay(1000);
}


void raiseClaw() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  delay(1500); // Adjust based on how long it takes to raise
  stopClawMotor();
}


  // Does mission, will implement later
  // Returns true if puck is magnetic, false if it's not
  bool do_mission(){
    // Implement exact precise positioning
    // Implement using the arm (motor & servo)
    // Implement hall sensor code
    // Implement exact precise positioning
    // Implement using the arm (motor & servo)
    // Implement hall sensor code
   
    lowerClawToGround();


    //Check for magnetic puck
    puckDetected = digitalRead(hallSensorPin) == LOW;
    bool magnetic = puckDetected;


  // 3. Close the claw
    closeClaw();


  // 4. Raise the claw
    raiseClaw();


    // You're at A
    if (getY() > 1){
      moveTo(0.55, 0.55);
    }
    else{
      moveTo(0.55, 1.45);
    }




    return puckDetected;




    // You're at A
    if (getY() > 1){
      moveTo(0.55, 0.55);
    }
    else{
      moveTo(0.55, 1.45);
    }


    return true;
  }




  // Returns whether or not there's a wall right in front
  bool wall_in_front(){
     digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);


 
    long duration = pulseIn(echoPin, HIGH);


 
    int distance = duration * 0.034 / 2;


    return distance <= distanceThreshold;


  }


  int which_case(){
    // Bottom wall is close
    if (walls[0]){
      // Middle wall is close    therefore top wall is far
      if (walls[1]){
        return 3;
      }
      // Middle wall is far
      else{
        // Top wall is close
        if (walls[2]){
          return 6;
        }
        // Top wall is far
        else{
          return 4;
        }
      }
    }
    // Bottom wall is far
    else{
      // Middle wall is close
      if (walls[1]){
        // Top wall is close
        if (walls[2]){
          return 1;
        }
        // Top wall is far
        else{
          return 5;
        }
      }
      // Middle wall is far     therefore top wall is close
      else{
        return 2;
      }
    }
  }








  // Phase 2    Robot starts at approx (x, y) = (1, 0.5) theta = 0
  bool nav_past_walls(){


    // True if wall is closer, false if it's farther. Bottom wall is 0, middle is 1, top is 2
    walls[0] = wall_in_front();
    moveTo(1, 1);
    walls[1] = wall_in_front();
    moveTo(1, 1.5);
    walls[2] = wall_in_front();
    case_num = which_case();
   
    // You start at (x, y) = (1, 1.5) theta = 0. Code all 6 cases.
    switch(case_num) {
      case 1:
        // code block for case 1  (done)
        moveTo(1, 0.5);
  	delay(1000);
        moveTo(1.9, 0.5);
  	delay(1000);
        moveTo(1.9, 1);
  	delay(1000);
        moveTo(2.8, 1); // You're now at the open zone
 	 delay(1000);
        break;
      case 2:
        // code block for case 2  (done)
        moveTo(1, 1);
delay(1000);
        moveTo(1.9, 1);
delay(1000);
        moveTo(1.9, 1.5);
delay(1000);
        moveTo(2.8, 1.5); // You're now at the open zone
delay(1000);
        break;
      case 3:
        // code block for case 3  (done)
        moveTo(1, 1.5);
delay(1000);
        moveTo(1.9, 1.5);
delay(1000);
        moveTo(1.9, 1);
delay(1000);
        moveTo(2.8, 1); // You're now at the open zone
	delay(1000);
        break;
      case 4:
        // code block for case 4  (done)
        moveTo(1, 1.5);
	delay(1000);
        moveTo(1.9, 1.5);
	delay(1000);
        moveTo(1.9, 0.5);
	delay(1000);
        moveTo(2.8, 0.5); // You're now at the open zone
	delay(1000);
        break;
      case 5:
        // code block for case 5  (done)    same as case 3
        moveTo(1, 1.5);
	delay(1000);
        moveTo(1.9, 1.5);
	delay(1000);
        moveTo(1.9, 1);
	delay(1000);
        moveTo(2.8, 1); // You're now at the open zone
	delay(1000);
        break;
      case 6:
        // code block for case 6  (done)    same as case 2
        moveTo(1, 1);
	delay(1000);
        moveTo(1.9, 1);
	delay(1000);
        moveTo(1.9, 1.5);
	delay(1000);
        moveTo(2.8, 1.5); // You're now at the open zone
	delay(1000);
        break;
      default:
        moveTo(2.8, 0.5); // We are facing the log
        // simple_turn(0);
    }














  }


  bool no_sensor_past_walls(){


    // True if wall is closer, false if it's farther. Bottom wall is 0, middle is 1, top is 2
    walls[0] = wall_in_front();
    moveTo(1, 1);
    walls[1] = wall_in_front();
    moveTo(1, 1.5);
    walls[2] = wall_in_front();
    case_num = 1;   // Can test 1 or 2
   
    // You start at (x, y) = (1, 1.5) theta = 0. Code all 6 cases.
    switch(case_num) {
      case 1:
        // code block for case 1  (done)
        moveTo(1, 0.5);
        force_update(1, 0.5, 0);
        moveTo(1.9, 0.5);
        force_update(1.9, 0.5, 0);
        moveTo(1.9, 1);
        force_update(1.9, 1, 0);
        moveTo(2.8, 1); // You're now at the open zone
        force_update(2.8, 1, 0);
        break;
      case 2:
        // code block for case 2  (done)
        moveTo(1, 1);
        force_update(1, 1, 0);
        moveTo(1.9, 1);
        force_update(1.9, 1, 0);
        moveTo(1.9, 1.5);
        force_update(1.9, 1.5, 0);
        moveTo(2.8, 1.5); // You're now at the open zone
        force_update(2.8, 1.5, 0);
        break;
      case 3:
        // code block for case 3  (done)
        moveTo(1, 1.5);
        moveTo(1.9, 1.5);
        moveTo(1.9, 1);
        moveTo(2.8, 1); // You're now at the open zone
        break;
      case 4:
        // code block for case 4  (done)
        moveTo(1, 1.5);
        moveTo(1.9, 1.5);
        moveTo(1.9, 0.5);
        moveTo(2.8, 0.5); // You're now at the open zone
        break;
      case 5:
        // code block for case 5  (done)    same as case 3
        moveTo(1, 1.5);
        moveTo(1.9, 1.5);
        moveTo(1.9, 1);
        moveTo(2.8, 1); // You're now at the open zone
        break;
      case 6:
        // code block for case 6  (done)    same as case 2
        moveTo(1, 1);
        moveTo(1.9, 1);
        moveTo(1.9, 1.5);
        moveTo(2.8, 1.5); // You're now at the open zone
        break;
      default:
        moveTo(2.8, 0.5); // We are facing the log
        force_update(2.8, 0.5, 0);
        // simple_turn(0);
    }


  }




  // Phase 3    Robot goes past log
  bool nav_log(){
    moveTo(3, 0.5);
    // simple turn(0);
    pwrMotors(255, 255, 255, 255);
    delay(10000);
    stop();


    // We did it!!!!!!!
  }










   
   
   
   
};






















// Forwards is 1.25m / 10,000 ms
// Move forwards with x power
void forwards(int amount) {
    pwrTL(amount);
    pwrTR(amount);
    pwrBR(amount);
    pwrBL(amount);
}
void rightwards(int amount) {
    pwrTL(-amount);
    pwrBL(amount);
    pwrTR(amount);
    pwrBR(-amount);
}
// Turn right
void turnRight(int amount){
    pwrTL(amount);
    pwrTR(-amount);
    pwrBL(amount);
    pwrBR(-amount);
}
// Turn left
void turnLeft(int amount){
    turnRight(-amount);
}
void stop(){
  forwards(0);
}


// The following 3 functions to turn & rotate
// Returns amount you need to turn given a goal angle. -PI < theta < PI
double turnAmount(double current_theta, double goalTheta){
  double amountTurn = (current_theta - goalTheta);
  while (amountTurn > 3.1415){
    amountTurn -= 6.283;
  }
  while (amountTurn < -3.1415){
    amountTurn += 6.283;
  }
  return amountTurn;
}


// Turns by amount in radians
void simple_turn (double amountTurn) {
  // Turn right if amountTurn < 0
  if (amountTurn > 0){
    turnRight(127);
  }
  // Turn left if amountTurn > 0
  else {
    turnLeft(127);
  }
  delay(amountTurn * 1909.85);
  stop();
}




// Rotates by very small amnt of radians    positive to go left, negative to go right
void adjustRotate(double amount){
  // Rotate left if amount < 0
  if (amount < 0) {
    turnRight(128);
    delay(50);
  }
  else{
    turnLeft(128);
    delay(50);
  }
  stop();
}


// Will be used to calculate velocity & distance
class Vector{
  public:
  double x, y;




  // Initializes a vector
  Vector(double a, double b){
    x = a;
    y = b;
  }




  // Returns magnitude of vector
  double magnitude(){
    return sqrt(x*x + y*y);
  }




  Vector rotation_matrix(double theta) {
    double a, b;
    a = x*cos(theta) - y*sin(theta);
    b = x*sin(theta) + y*cos(theta);
    return Vector(a, b);
  }
  // Return scaled vector by some constant
  Vector scaled(double value){
    return Vector(x * value, y * value);
  }


  Vector addition(Vector other){
    return Vector(x + other.x, y + other.y);
  }


  // Return normalized the vector to get a unit vector output
  Vector normalized()  {
    double mag = magnitude();
    if (mag == 0){
      return Vector(0, 0);
    }
    return Vector(x / mag, y / mag);
  }
};


// Let x and y be in cm, theta in radians.
void vector_to(double curr_x, double curr_y, double curr_theta, double x_dest, double y_dest){
  double x = curr_x; // Enes100.getX();
  double y = curr_y; // Enes100.getY();
  double theta = curr_theta; //Enes100.getTheta();


  // Calculate v using eye in the sky
  Vector v(x_dest - x, y_dest - y);


  // Calculate u (adjusted with theta) using the rotation matrix.
  // Calculate delta theta using turnAmount() function
  Vector w = v.rotation_matrix(theta);


  // Delay depending on distance needed to travel     depends on speed
  double delay_amnt = 200.0 * testSpeed;  // Inverse of speed      ms/cm    speed @255 = 0.0125cm/ms




  int m1 = w.x; // Front/back distance
  int m2 = w.y * 1.05; // Sideways distance   (multiplied by 1.05 since sideways is slower than forwards)


  // Go forwards first
  if (m1 < 0){
    forwards(-255);
    delay(delay_amnt * -m1);
  }
  else{
    forwards(255);
    delay(delay_amnt * m1);
  }  
  stop();
  // Go sideways after
  if (m2 < 0){
    rightwards(-255);
    delay(delay_amnt * -m2);
  }
  else{
    rightwards(255);
    delay(delay_amnt * m2);
  }  
  stop();
}






void loop() {
  for (int i = 1; i < 10; i++){
    // eighth turns
    simple_turn(i * PI/8);
    delay(4000);
  }
 
