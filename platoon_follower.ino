// Platoon Project Following Car
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

#define servoPin 7 // pin for servo signal
#define rcSteeringPin 25
#define rcThrottlePin 23
#define rcGrndPin 27
#define sdSelectPin 49
#define motorPin 8 // PWM for motor
#define SLAVE_ADDRESS 0x04
//
// LSM9DS1 9-DOF with I2C
//
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega
//
// servo
//
Servo steeringServo;
float servoBiasDeg = 80.0;

float gpsPsi;
float frontEndL = 0.15; //m
float Pi = 3.1416;
float widthcamera = 0.2; //units???
  byte betaimage = 20;
//float headingconstant = 0.1
float heading_est_degrees = 0;
float offset_est = 0;

//motor contants
float pwm0 = 150/255; //experimentally determine pwm0 for DC 
float v_max = 5; //maximum safe velocity for car, [m/s]
float c_accel = 30; //acceleration-to-PWM conversion, experiments needed [s], positive
float c_p = 1/70; //DC motor constant, must be experimentally determined, [(m/s)/(PWM%)]
float k_delta = -v_max/90; // [(m/s)/degree] ...no penalty at zero turn angle, max penalty at 90 degree turn
float speed_estimate = 0; //how to derive this?

//I2C
byte piCommand;
String value;
String offset;




void setup() {
    Serial.begin(115200);
    Serial.println("PlatoonFollowerCar");  
//
// IMU
//
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

//
//  set up SD card data log
//
    if (!SD.begin(sdSelectPin)) {
        Serial.println("SD Card fail");
        while(1);
    }
    File dataFile = SD.open("leadCar.csv", FILE_WRITE);
    if (dataFile) {
       dataFile.println("Time [ms] , Gyro Z [deg/sec] , Accel Y [m/s/s] , RC Throttle [pct] , RC Steering [pct] , Motor PWM [byte] , Servo Angle [deg]");
       dataFile.close();
    }

//
//  set up the rc control pins
//
    pinMode(rcGrndPin,OUTPUT); digitalWrite(rcGrndPin,LOW);
    pinMode(rcSteeringPin,INPUT);
    pinMode(rcThrottlePin,INPUT);
//
//  make the motors not spin
//
    pinMode(motorPin,OUTPUT);
    analogWrite(motorPin,0);
//
//  connect the steering servo
//  send bias
//
    steeringServo.attach(servoPin);
    steeringServo.write(servoBiasDeg);

    lsm.begin();
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveDataI2C);
    Wire.onRequest(sendDataI2C);
  }
}









//////////////////////////////////////////////////////////////////
void loop() {
//
// IMU
//
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 
  Serial.print("AY: "); Serial.print(a.acceleration.y);
  Serial.print(" GyroZ: "); Serial.print(g.gyro.z); Serial.print(" ");
//
//pi will figure out and tell the distance to leader, 
//arduino should figure out relative linear and angular velocities of leader, and replicate those with control
//  read the RC commands
//
//    unsigned long rcThrottleT = 0;
//    unsigned long rcSteeringT = 0;
//    rcThrottleT = pulseIn(rcThrottlePin,HIGH);
//    rcSteeringT = pulseIn(rcSteeringPin,HIGH);
//    float rcThrottlePct = constrain((rcThrottleT-1000.0)/8.0,0.0,100.0);
//    float rcSteeringPct = constrain((rcSteeringT-1500.0)/4.0,-100.0,100.0);
//    Serial.print(rcThrottlePct); Serial.print(" ");
//    Serial.print(rcSteeringPct); Serial.print(" ");
//
//  set motor command
//  estimate the corresponding speed of the car
//

    float rcThrottlePct= ;
    float rcSteeringPct= ;
    byte motorPWM = constrain(2.5*rcThrottlePct,0,255);
    analogWrite(motorPin,motorPWM);
    Serial.print(motorPWM);
    Serial.print(" ");
    float estimatedV = constrain(0.6*(motorPWM - 50),10,120); // cm/sec
//
//  generate a commanded response--how fast do we want the car to turn
//  estimate the control required to make it turn that fast
//
    float commandedPsiDot = 0.7*rcSteeringPct;  // desired turn rate in deg/sec
    float servoEstimateDeg = commandedPsiDot*(17.0/estimatedV); // feed-forward steering to get desired turn rate
//
//  psi_error = washout of integrated yaw rate error
//  (also, reset when car not moving so you can pick up and rotate)
//
    static float psi_error = 0.0;
    static float millisLast = millis();
    float tauPsi = 2.0; // washout time constant, in seconds
    psi_error += (-(1/tauPsi)*psi_error + (commandedPsiDot + g.gyro.z))*(millis() - millisLast)*0.001;
    millisLast = millis();
    psi_error = constrain(psi_error,-10,10); // don't let the heading error get too big -- it might cause problems
    if (rcThrottlePct < 5) psi_error = 0.0; // reset the heading error when we aren't moving
    Serial.print(psi_error); Serial.print(" , ");

    float Kpsi = 2.0;
    float servoAngleDeg = constrain(servoBiasDeg+servoEstimateDeg+Kpsi*psi_error,servoBiasDeg-30,servoBiasDeg+30);
    steeringServo.write(servoAngleDeg);
    Serial.print(servoAngleDeg); Serial.print(" ");
//
//  log the data if throttle is on
//
    if (rcThrottlePct > 5) {
        File dataFile = SD.open("leadCar.csv", FILE_WRITE);
        if (dataFile) {
           dataFile.print(millis()); dataFile.print(" , ");
           dataFile.print(g.gyro.z); dataFile.print(" , ");
           dataFile.print(a.acceleration.y); dataFile.print(" , ");
           dataFile.print(rcThrottlePct); dataFile.print(" , ");
           dataFile.print(rcSteeringPct); dataFile.print(" , ");
           dataFile.print(motorPWM); dataFile.print(" , ");
           dataFile.println(servoAngleDeg);
           dataFile.close();
        }
    }
//
//  finish the dump of information to the Serial monitor
//
    Serial.println();
//
//  reading the RC commands will cause effective pause of 20+ ms
//  so we don't really need a pause here...
//
    delay(1);
}

void receiveDataI2C(int nPoints) {
      piCommand = Wire.read();
      //
      // if Pi is sending data, parse it into incoming data array
      //
      if (piCommand == 255) {
          byte length = Wire.read();
         for (int i = 0; i < length; i++){
             offset+=Wire.read();
         }
         byte transValue[length];
         String tempString;
         for (int i = 0; i < length; i++) {
             int temp = (value[2*i]-'0')*10 + (value[2*i+1]-'0');
             if (temp >= 48){
                tempString += String(temp-'0');
             } else{
                tempString += '.';
             }            
         }
         widthcamera = tempString.toFloat();
//         Serial.println(finalValue);
         offset = "";
      }
      else if (piCommand == 254) {
         byte length = Wire.read();
         for (int i = 0; i < length; i++){
             value+=Wire.read();
         }
         byte transValue[length];
         String tempString;
         for (int i = 0; i < length; i++) {
             int temp = (value[2*i]-'0')*10 + (value[2*i+1]-'0');
             if (temp >= 48){
                tempString += String(temp-'0');
             } else{
                tempString += '.';
             }            
         }
         betaimage = tempString.toFloat();
         value = "";
      }

      while (Wire.available()) {Wire.read();}
}

void sendDataI2C(void) {
    if (piCommand == 1) {
        float dataBuffer[3];
        dataBuffer[0] = 3.5;//estHeading;
        dataBuffer[1] = 0.2;//estOffset;
        dataBuffer[2] = 1.0;//estSpeed
        Wire.write((byte*) &dataBuffer[0], 3*sizeof(float));
    }

}
