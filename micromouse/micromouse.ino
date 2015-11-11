#include <Wire.h>
#include <SparkFun_VL6180X.h>

#define VL6180X_ADDRESS 0x29

VL6180xIdentification identification;
VL6180x sensor(VL6180X_ADDRESS);

// Question: What is this??
int leftSpeed = 4;
int rightSpeed = 5;

//Motor pins
int leftMotorPinA = 6;
int leftMotorPinB = 7;
int rightMotorPinA = 8;
int rightMotorPinB = 9;

// sensor pins
int frontSensorPin = 14; // using the sharp IR sensor
int rightSensorPin = 15; // using the ToF VL6180X sensor
int frontSensorRead;
int frontBlock = 100; // define some threshold value for the mouse to detect an obstacle in front
int rightBlock = 180; //define some threshold value for the mouse to detect an obstacle on the right. 180mm = 18cm

// Encoder pins (to detect the exact distance travelled)
int leftEncoderPinA = 10;
int leftEncoderPinB = 9;
int rightEncoderPinA = 12;
int rightEncoderPinB = 11;

//need to measure how much distance corresponds to 1 tick
volatile int leftTicks = 0;
volatile int rightTicks = 0;

// other variables
long debounce = 500;
long obstacleDetect = 0;



void setup() {
    Serial.begin(115200); //Start Serial at 115200bps
    Wire.begin(); //Start I2C library
    delay(100); // delay .1s

    sensor.getIdentification(&identification); // Retrieve manufacture info from device memory
    if(sensor.VL6180xInit() != 0){
      Serial.println("FAILED TO INITALIZE"); //Initialize device and check for errors
    };
    sensor.VL6180xDefautSettings(); //Load default settings to get started.

 // initializing the speeds of the left and right motor
    driveforwardLeft(1);
    driveforwardRight(1);
    
    pinMode(leftMotorPinA, OUTPUT);
    pinMode(leftMotorPinB, OUTPUT);
    pinMode(rightMotorPinA, OUTPUT);
    pinMode(rightMotorPinB, OUTPUT);
    
//What is this?? To increase the number of ticks when detected
    attachInterrupt(leftEncoderPinA, onLeftTick, CHANGE);
    attachInterrupt(leftEncoderPinB, onLeftTick, CHANGE);
    attachInterrupt(rightEncoderPinA, onRightTick, CHANGE);
    attachInterrupt(rightEncoderPinB, onRightTick, CHANGE);
    
 //move forward
    digitalWrite(leftMotorPinA, HIGH);
    digitalWrite(leftMotorPinB, LOW);
    digitalWrite(rightMotorPinA, HIGH);
    digitalWrite(rightMotorPinB, LOW);
    
}

int prev_error = 0;
void loop() {

 // checking distance from wall
    int error = 20 - sensor.getDistance(); //sensor.getDistance returns the distance in mm. In this case, we want the mouse to be 20mm from the wall. 
    int delta_error = error - prev_error;

// PID control
    float P = .01;
    //float D = -100;
    float pidOutput = P * error;
    //float pidOutput = P * error + D * delta_error;

    prev_error = error;
    
//    if (pidOutput > 1) {
//      pidOutput = 1;
//    } else if (pidOutput < -1) {
//      pidOutput = -1;
//    }
//Serial information for troubleshooting    
    Serial.print("Left: ");
    Serial.println(0.75 + 0.25 * pidOutput);
    Serial.print("Right: ");
    Serial.println(0.75 - 0.25 * pidOutput);
    
//Just moving straight, following a wall
    driveforwardLeft(0.30 + 0.1 * pidOutput);
    driveforwardRight(0.30 - 0.1 * pidOutput);
  




// checking for dead end:
  frontSensorRead = analogRead(frontSensorPin);
  Serial.println(frontSensorRead);
  
  
  // Logic: if deadend --> call deadEnd function. But only start responding if the obstacle is still there after a prolonged period. aka create a "debounce". This way, we can filter out noise
  // Backtracks until it senses that the road on the right is clear
  if (frontSensorRead <= frontBlock && sensor.getDistance() <= rightBlock  ){

    //backtrack until an opening on the right is detected
    while(sensor.getDistance()<= rightBlock){
      drivebackwardLeft(0.30 + 0.1 * pidOutput);
      drivebackwardRight(0.30 - 0.1 * pidOutput);
    }
    //backtrack a little more so we have more space
    turnRight(); // a function to make the mouse turn 90 degrees right
  }  
  // Logic: if not deadend --> call function to track wall and move forward



}

// Driving forward
       void driveforwardLeft(float spd) {
          analogWrite(leftSpeed, spd*255);
        // Question: why did Roy/Chandler comment out the below sections?
        //    analogWrite(leftMotorPinA, spd * 255);
        //    analogWrite(leftMotorPinB, 0);
        }
        
        void driveforwardRight(float spd) {
          analogWrite(rightSpeed, spd*255);
        //    analogWrite(rightMotorPinA, spd * 255);
        //    analogWrite(rightMotorPinB, 0);
        }

// Driving backwards
        void drivebackwardLeft(float spd) {
          analogWrite(leftSpeed, spd*255);
        // Question: why did Roy/Chandler comment out the below sections?
        //    analogWrite(leftMotorPinB, spd * 255);
        //    analogWrite(leftMotorPinA, 0);
        }
       
        void drivebackwardRight(float spd) {
          analogWrite(rightSpeed, spd*255);
        //    analogWrite(rightMotorPinB, spd * 255);
        //    analogWrite(rightMotorPinA, 0);
        }
        
// turning 90 deg right (assuming no slip)
  void turnRight(){
    long storeRightTicks = rightTicks;
    long storeLeftTicks = leftTicks;
    
    //pivot to right while moving forward
    while(leftTicks - storeLeftTicks < 50){
      analogWrite(leftMotorPinA, 100);
      analogWrite(leftMotorPinB, 0);

    }
    //pivot to left while moving backwards
    while(rightTicks - storeRightTicks < 50){ // To-do: as a prototype, i put '50' ticks. But I must fine-tune this number
      analogWrite(rightMotorPinA, 0);
      analogWrite(rightMotorPinB, 100);
    }
  }


// For rotorary encoder to detect ticks
void onLeftTick() {
    leftTicks++;
}

void onRightTick() {
    rightTicks++;
}

