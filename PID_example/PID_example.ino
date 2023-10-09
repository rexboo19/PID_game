#include <Servo.h>
#include <Wire.h>

////////////////////////Variables///////////////////////

// Ultrasonic sensor
//int pos = 0;
int trigPin = 2;
int echoPin = 3;

// Tilting Sensor
int tiltPin = 4;
int tilt_val = 0;
int microbit = 0;
int microbitPin = 5;
int levelOffset = 0;
//int Read = 0;


// Servo motor
int servoOffset = 95;
int servoPin = 9;
int setpointMotor;

///////////////////PID constants///////////////////////

float elapsedTime, time, timePrev;        //Variables for time control

float distance_setpoint = 17;        //Should be the distance from sensor to the middle of the bar in cm  
float K = 0.5; //Scaling 
float kp=1.2; //Mine was 8 //6
float kd=0.5; //Mine was 3100 //3100
float ki=1; //Mine was 0.2 //0.6
float PID_p, PID_i, PID_d, PID_total;

float pError = 0;
float iError = 0;
float dError = 0;
float previousError;

float distance = 0.0;
float last_distance = 0.0;
float difference;
float previousDifference = 0;

float period = 0.01;  //Refresh rate period of the loop is 50ms

float buff_distance, buff_distance1, buff_distance2, buff_distance3, buff_distance4, buff_distance5;
///////////////////////////////////////////////////////

Servo myservo;  // create servo object to control a servo

void setup() {
  time = millis(); // Update the timer
  
  myservo.attach(servoPin);
  myservo.write(servoOffset); //Center of servo
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(tiltPin, INPUT);
  pinMode(microbitPin, INPUT);
  
  Serial.begin(115200);

  buff_distance2 = distance_setpoint;
  buff_distance1 = distance_setpoint;
  
}

void loop() {

  //Update the system after a period of time
  if (millis() > time+period){
    time = millis();  //Update the timer

    /***  Distance validation ***/

    //Update the current distance of the ball to the sensor.
    
    // Limit the range of the returned distance value,
    // If the distance is <= 0, it would be an error of detecting
    // the distance, set the distance to the last updated distance;
    // If the distance detected is >30, the dectection is out of range
    // and set the distance to the last updated distance.

    // 3 Frames for digital low pass filtering
    buff_distance3 = buff_distance2;
    buff_distance2 = buff_distance1;
    
    buff_distance = get_distance();
    
    if(buff_distance <=0){ //Error distance value
      buff_distance1 = last_distance; //Ignore the error value
    }else if(buff_distance >30){ //The sensor value beyond the distance range
      buff_distance1 = 30;
    }else{  //The distance is within the valid range
      buff_distance1 = buff_distance; //Update the distance to the current sensor value
    }
    distance = (buff_distance1+buff_distance2+buff_distance3)/3; // Averaging the distance to remove noise
    
    last_distance = distance; //Record the distance to be the last distance

    /***  Tilting Detection ***/
        
    tilt_val = digitalRead(tiltPin);
    microbit = digitalRead(microbitPin);

    //////////////////////////////////////////////////////////////////////////
    /***  PID computation ***/

    // Calculate the current error //
    pError = distance_setpoint - distance;
    PID_p = kp * pError;

    
    

    //////////////////////////////////////////////////////////////////////////

    // Calculate the differential error //
    difference = pError - previousError;
    
    if (abs(difference) < 0.5){ //Remove the noise of rate Error
      difference = 0;
      previousDifference = difference;
    }
    if (abs(difference) > 10){ //Remove the noise of rate Error
      difference = previousDifference;
    }
    previousDifference = difference;
    
    difference = constrain(difference, -10, 10); //Limit the size of the rate error
    dError = difference/period;
    
    PID_d = kd * dError;
    
    previousError = pError;

    

    //////////////////////////////////////////////////////////////////////////

    // Calculate the intergral error //
    //The effect of intergral error only effective when it is closed to the setpoint,
    //Otherwise, it does not affect the control.
    if(-10 < pError && pError < 10) {
      iError = iError + (pError * period);
    }else{
      iError = 0;
    }
    if (iError > 30){iError = 0;}
    
    PID_i = ki * iError;

    //////////////////////////////////////////////////////////////////////////

    PID_total = (PID_p + PID_i + PID_d) * K;  
    PID_total = constrain(PID_total, -20, 20); //Limit the size of the PID

    //If flat serface, levelOffset = 0
    levelOffset = 0;
    setpointMotor = servoOffset + PID_total + levelOffset;
    setpointMotor = constrain(setpointMotor, 60, 140); //Limit the size of the servo motor
    myservo.write(setpointMotor);

    infoLog();
  }
}

void infoLog(){
  Serial.print("Setpoint: ");
  Serial.print(distance_setpoint);
  Serial.print(", Sensor distance: ");
  Serial.print(distance);
  //Serial.print(", last distance: ");
  //Serial.print(last_distance);

  Serial.print(", Servo motor: ");
  Serial.print(setpointMotor);

  Serial.print(", PID_p: ");
  Serial.print(PID_p);
  Serial.print(", PID_i: ");
  Serial.print(PID_i);
  Serial.print(", PID_d: ");
  Serial.print(PID_d);
  Serial.print(", PID: ");
  Serial.print(PID_total);
  
  Serial.println("");
}
float get_distance()
{
  int duration;
  int distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration *0.034/2;
  return distance; 
}
