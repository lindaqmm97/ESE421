// Platoon Project Following Car
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

#define SLAVE_ADDRESS 0x04
#define servoPin 7 // pin for servo signal
#define motorPin 8 // PWM for motor
#define pingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingGrndPin 27 // ping sensor ground pin (use digital pin as ground)
float cmold = 0;
int offsetdes = 0;

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega

// servo
Servo steeringServo;
float servoBiasDeg = 80.0;

float gpsPsi;
float Pi = 3.1416;
float widthcamera = 0.2; //units???
byte betaimage = 20;    //how to read in???
byte offsetimage = 0;
float timeRead=20; //[ms]
float frontEndL = 0.15; //m
int k_d = 0.5; //[s], converts leader speed to desired following distance, test this
int k_ed = 0.7; //distance error control
int k_off = 0.7; //offset control
int k_beta = 0.7;  //beta control
int tauPing = 0;
int tauCam = 0;

//motor contants
float pwm0 = 100/255; //experimentally determine pwm0 for DC 
float v_max = 50; //maximum safe velocity for car, [cm/s]
float c_p = 1/7; //DC motor constant, must be experimentally determined, [(cm/s)/(PWM%)]
float k_delta = -v_max/90; // [(cm/s)/degree] ...no penalty at zero turn angle, max penalty at 90 degree turn
float estimatedV = 0;

//I2C
byte piCommand;
String value;
String offset;

/////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(115200);
    Serial.println("PlatoonFollowerCar");  
// IMU
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

//  set up SD card data log
    if (!SD.begin(sdSelectPin)) {
        Serial.println("SD Card fail");
        while(1);
    }
    File dataFile = SD.open("leadCar.csv", FILE_WRITE);
    if (dataFile) {
       dataFile.println("Time [ms] , Gyro Z [deg/sec] , Accel Y [m/s/s] , RC Throttle [pct] , RC Steering [pct] , Motor PWM [byte] , Servo Angle [deg]");
       dataFile.close();
    }

//  make the motors not spin
    pinMode(motorPin,OUTPUT);
    analogWrite(motorPin,0);

//  connect the steering servo
//  send bias
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

// IMU
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 
  Serial.print("AY: "); Serial.print(a.acceleration.y);
  Serial.print(" GyroZ: "); Serial.print(g.gyro.z); Serial.print(" ");
  
//use ping to find distance
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:
  long duration, cm;
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  // convert the time into a distance
  float cmnew = microsecondsToCentimeters(duration);
  // if cmnew reads something weird, disregard it and use the camera value?
  //high pass filter both camera and ping and then average them?
  distance = cmnew; //(for now)
      
      
//use current estimated speed, timestep, and  distance difference to estimate leader speed
  float leaderV = estimatedV+(distnew-distold)/timeRead;
  float distdes = k_d*leaderV; //scale factor for desired distance, *s, 50cm/s gives desired distance of 25cm
//Emergencystop  
    if (distnew <= distdes){
        analogWrite(motorPin,0);
  }
  
//command motor to fix distancing, convert to PWM, estimate current velocity from that
  float commandedV = estimatedV + (distdes - cmnew)*k_ed
  byte motorPWM = pwm0 + c_p*commandedV;
  distold= distnew;
  analogWrite(motorPin,motorPWM);
  Serial.print(motorPWM);
  Serial.print(" ");
  estimatedV = constrain(0.6*(motorPWM - 50),10,120); // cm/sec
  
//read in betaimage and offset, filter both
  //integrate and high-pass filter gyro, low-pass filter the camera beta
  float betafiltered = beta
  //low-pass filter the offset
  float offsetfiltered = offset
    
//find delta from those
  float commandeddelta = k_beta*(k_off*(offsetdes - offsetfilt) - betafiltered) + servoBiasDeg;
  float servoAngleDeg = constrain(commandeddelta,servoBiasDeg-30,servoBiasDeg+30);
  steeringServo.write(servoAngleDeg);
  Serial.print(servoAngleDeg); Serial.print(" ");
  
  Serial.println();
}


///////////////////////////////////////
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

/////////////////////////////////////////////////////
void sendDataI2C(void) {
    if (piCommand == 1) {
        float dataBuffer[3];
        dataBuffer[0] = 3.5;//estHeading;
        dataBuffer[1] = 0.2;//estOffset;
        dataBuffer[2] = 1.0;//estSpeed
        Wire.write((byte*) &dataBuffer[0], 3*sizeof(float));
    }

}

//////////////////////////////////////////////////////
long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}
