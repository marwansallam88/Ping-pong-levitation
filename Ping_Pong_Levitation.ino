
const int pm = A0; //potenmeter position
const int trigPinBall = 6; //ball position ultrasonic sensor Trig
const int echoPinBall = 5; //ball position ultrasonic sensor Echo
const int fanPin      = 3;// PWM output for motor control
// defines variables
unsigned long durationB;
double distanceB;
double ref = 0; //position of hand
double ballPos = 0; //position of the ball in the tube
//Geometrical parameters of the system
const double upperGap = 10; //upper gap in cm
const double lowerGap = 10; //lower gap in cm
const double columnL = 30; //effective length of the tube in cm
#define PREV_REF 70;
double previousRef = PREV_REF; // the initial desired reference is PREV_REF [%] of the effective length of the tube
const double ballDiam = 4; //diameter of the ball in cm
const int maxWaitTime = (int) (columnL * 3.5 / 0.034); //maximal waiting time for the ultrasonic sensors to receive the reflected wave
//variables for the control loop
double controlP = 0;
double controlI = 0;
double controlD = 0;
double control = 0;
double err = 0;
double prevErr = 0;
double previousBallPos = 0; //the initial ball position, then updates
double prevControlI = 0; //initial condition for integrator

//PID constants
const double samplingTime = 0.0625; //sampling time in seconds

const double intTimea = 0.4 * 2.0;                           
const double difTimea = 0.02 * 0.77; 
const double Kpa = 0.7 * 3;                            
const double Kia = Kpa / intTimea;
const double Kda = Kpa * difTimea;

const double intTimeb = 0.50 * 3.2;
const double difTimeb = 0.012 * 2.2; 
const double Kpb = 0.35 * 2.0; 
const double Kib = Kpb / intTimeb;
const double Kdb = Kpb * difTimeb;

const double intTimec = 0.5 * 3.2; 
const double difTimec = 0.014 * 1.1;
const double Kpc = 0.45 * 1.65;
const double Kic = Kpc / intTimec;
const double Kdc = Kpc * difTimec;
//flags
int flag = 0; //used as a counter -> decides when to write to output
bool flagRefErr = 0; //true if there is an error reading reference
bool flagBallPosErr = 0; //true if there is an error reading ball position
char region = 'a'; //region 'a', 'b' or 'c' - division of the tube into three regions
//Limits for regions
const int limitAB = 18; //in % of the effective length of the tube
const int limitBC = 64; //in % of the effective length of the tube

void setup() {
  pinMode(trigPinBall, OUTPUT); // Sets the trigPin as an Output for Ball position sensor
  pinMode(echoPinBall, INPUT); // Sets the echoPin as an Input for Ball position sensor
  pinMode(pm, INPUT); // Sets the potentiometer as an Input for ref position sensor
  pinMode(fanPin, OUTPUT); //Sets the fanPin to an output mode
  Serial.begin(9600); // Starts the serial communication
  Serial.println("ballPos,ref, control");
}



void loop() {
  
  region = 'a';



    //____________ref___________________
      ref = analogRead(pm);
      ref = ref/1023;
      ref = ref*100;



    //_____________BALL_________________
    digitalWrite(trigPinBall, LOW); // Clears the trigPin
    delayMicroseconds(2);
    digitalWrite(trigPinBall, HIGH);// Sets the trigPin on HIGH state for 10 microseconds
    delayMicroseconds(10);
    digitalWrite(trigPinBall, LOW);
    durationB = pulseIn(echoPinBall, HIGH, maxWaitTime);// Reads the echoPin, returns the sound wave travel time in microseconds
    // Calculating the distance
    distanceB = durationB * 0.034 / 2;
    
    if (distanceB == 0) //Handling the error
    {
      flagBallPosErr = true;
      durationB = maxWaitTime;
    }
    else flagBallPosErr = false;
    ballPos = (columnL + upperGap) - (distanceB + ballDiam / 2); // scales the ball position value in the reference system
    delay((int)(25 - durationB * 0.001));
    
    


    //scale the values from 0 to 100
    ballPos = ballPos * (100 / columnL);
    if ((ballPos < 0) || (ballPos > 100) || flagBallPosErr) ballPos = previousBallPos;

    
    
    //Anti wind-up solution for transitions between different subregions of the tube
    if (ballPos > limitAB && ballPos <= limitBC && previousBallPos <= limitAB) //transition from region A to B
    {
      prevControlI = control - Kpb * (ref - ballPos) - Kib * (ref - ballPos) * samplingTime - Kdb * (-ballPos + previousBallPos) / samplingTime; //bumpless control
      region = 'b';
    }
    else if (ballPos > limitBC && previousBallPos > limitAB && previousBallPos <= limitBC) //transition from region B to C
    {
      prevControlI = control - Kpc * (ref - ballPos) - Kic * (ref - ballPos) * samplingTime - Kdc * (-ballPos + previousBallPos) / samplingTime; //bumpless control
      region = 'c';
    }
    else if (ballPos > limitAB && ballPos <= limitBC && previousBallPos > limitBC) //transition from region C to B
    {
      prevControlI = control - Kpb * (ref - ballPos) - Kib * (ref - ballPos) * samplingTime - Kdb * (-ballPos + previousBallPos) / samplingTime; //bumpless control
      region = 'b';
    }
    else if (ballPos <= limitAB && previousBallPos > limitAB && previousBallPos <= limitBC) //transition from region B to A
    {
      prevControlI = control - Kpa * (ref - ballPos) - Kia * (ref - ballPos) * samplingTime - Kda * (-ballPos + previousBallPos) / samplingTime; //bumpless control
      region = 'a';
    }
    else if (ballPos > limitBC && previousBallPos <= limitAB) //transition from region A to C
    {
      prevControlI = control - Kpc*(ref-ballPos) - Kic*(ref-ballPos)*samplingTime - Kdc*(-ballPos+previousBallPos)/samplingTime; //bumpless control
      region = 'c';
    }

    else if (ballPos <= limitAB && previousBallPos > limitBC) //transition from region C to A
    {
      prevControlI = control - Kpa*(ref-ballPos) - Kia*(ref-ballPos)*samplingTime - Kda*(-ballPos+previousBallPos)/samplingTime; //bumpless control
      region = 'a';
    }

    PID(region); //calculates the control parameters for the current sampling interval
    //uses the PID to set the value of the power to the motor according to the position and speed
      motorPower(fanPin, control); //Control voltage sent to motor
      previousBallPos = ballPos; //stores prevouos value of ref in memory for next iteration
 Serial.print(ballPos);
 Serial.print(",");
 Serial.print(ref);
 Serial.print(",");
 Serial.println(control);
 
  delay(5);

}

//**************************************************************************************************************************************************

double getDistance (int trigPin , int echoPin) {
  // this function get the distance and save the value as output the inputs are the pin of the  US sensor
  unsigned long duration = 0;
  double distance = 0;
  digitalWrite(trigPin, LOW); // Clears the trigPin
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);// Sets the trigPin on HIGH state for 10 micro seconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);// Reads the echoPin, returns the sound wave travel time in microseconds
  // Calculating the distance
  distance = duration * 0.034 / 2;
  return distance;
}


void motorPower (int motorPin , double power) {
  //Function that writes the values of the desired power on the pin of the motor.
  //The inputs are the pin to which the motor is attached and the desired power (number 0-100).


  double powerOut = (int)((power / 100) * 255); //gets the fraction of power in input, number between 0 and 1
  analogWrite(motorPin, powerOut); //gives the voltage to the motor, an integer 0-255, proportional to the fraction of the power in input, 20 mV each step

}




void PID (char region) {
  //This function calculates the control parameters of the feedback.
  //No inputs. Calculates and stores variables.
  double Kp, Ki, Kd;
  if (region == 'a') //controller parameters for the lower part of tube
  {
    Kp = Kpa;
    Ki = Kia;
    Kd = Kda;
  }
  else if (region == 'b') //controller parameters for the middle part of tube
  {
    Kp = Kpb;
    Ki = Kib;
    Kd = Kdb;
  }
  else  if (region == 'c') //controller parameters for the upper part of tube
  {
    Kp = Kpc;
    Ki = Kic;
    Kd = Kdc;
  }

  err = ref - ballPos; //current error
  //calculates the proportional part
  controlP = Kp * err;
  //calculates the integral part
  controlI = prevControlI + Ki * err * samplingTime; //integral-->area
  //calculates the derivative part
  controlD = Kd * (-ballPos + previousBallPos) / samplingTime; //neglecting the reference, because of infinitive derivatives on step changes

  //calculates the total control
  control = controlP + controlI + controlD;
  if (control > 100) {//Technically impossible to actuate, but necessary to saturate in program...
    control = 100;
    controlI = prevControlI; //...to prevent integrator wind-up
  }
  else if (control < 0) {//Technically impossible to actuate, but necessary to saturate in program...
    control = 0;
    controlI = prevControlI; //...to prevent integrator wind-up in opposite direction
  }
  prevControlI = controlI; //stores the integrator value for the next iteration
  prevErr = err; //stores the value of the error for the next iteration
}
