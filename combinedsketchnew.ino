#include <Servo.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#define motorPin 8 // PWM for motor
#define pingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingGrndPin 27 // ping sensor ground pin (use digital pin as ground)

#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega
#define servoPin 7 // pin for servo signal

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);

Servo steeringServo;
byte servoAngleDeg = 90;
byte motorPWM=175;
byte motorPWMCurrent;
float pingDistanceCM = 0.0;
//byte gyroZBias=0;
//byte servoBias=0;
float timeRead=20;
float  k_heading =1.5;
//float GPSangle;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    HardwareSerial mySerial = Serial1;
    Adafruit_GPS GPS(&Serial1);
    pinMode(motorPin,OUTPUT);
    analogWrite(motorPin,0);
    pinMode(pingGrndPin,OUTPUT); digitalWrite(pingGrndPin,LOW);
    pinMode(pingTrigPin,OUTPUT);
    pinMode(pingEchoPin,INPUT);
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    steeringServo.attach(servoPin);
    steeringServo.write(servoAngleDeg);
//  read bias by taking the average of 10 gyro.z readings  
    lsm.begin();
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 

}

    SIGNAL(TIMER0_COMPA_vect) {
    char c = GPS.read();
}

void loop() {
  // put your main code here, to run repeatedly:
  int tau=4;
  static unsigned long timer = millis();
  motorPWMCurrent=motorPWM;
  getPingDistanceCM();
  if(pingDistanceCM<=30.0){
    motorPWMCurrent=0;
  } 
  analogWrite(motorPin,motorPWMCurrent);
  lsm.read();  /* ask it to read in the data */
  float gpsPsi = GPS.angle;
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  static float heading_est_degrees = 0;
  if(g.gyro.z>10 ||g.gyro.z<-10){
  heading_est_degrees -= (g.gyro.z)*(.001*timeRead);
  static float newAngle=0;
  newAngle+=(.001*timeRead)/tau*(gpsPsi-heading_est_degrees-newAngle);
  float psiEstimated=newAngle+heading_est_degrees;
  float servoAngle = servoAngleDeg-k_heading * psiEstimated; 
//  Serial.println(g.gyro.z);  
  steeringServo.write(constrain(servoAngle,servoAngleDeg-30, servoAngleDeg+30)); }
  while(millis()-timer<20){
  }
  timer=millis();
//  delay(15);
}

void getPingDistanceCM()
{
  //
  // 3000 us timeout implies maximum distance is 51cm
  // but in practice, actual max larger?
  //
  const long timeout_us = 3000;
  //
  // pingTrigPin = trigger pin
  // pingEchoPin = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  digitalWrite(pingTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingTrigPin, LOW);
  //
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //
  unsigned long echo_time;
  echo_time = pulseIn(pingEchoPin, HIGH, timeout_us);
  if (echo_time == 0)
 {
    echo_time = timeout_us;
 }
  //
  // return the distance in centimeters
  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  // divide by 2 because we measure "round trip" time for echo
  // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
  // = 0.017*echo_time
  //
  pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);
}
