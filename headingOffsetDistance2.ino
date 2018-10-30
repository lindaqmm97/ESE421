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
byte motorPWMCurrent=200;
float pingDistanceCM = 0.0;
float timeRead=20;
float k_heading =1.5;
float gpsPsi;
float servoAngle=90;
//newly added
float headingconstant = 0.1;
float frontEndL = 0.15;
float Pi = 3.1415;
float pwm0 = 15; //experimentally determine pwm0 for DC motor
//control constants
float k_width = -0.3;
float k_offset = 0.3; //converts offset change into an angle, geometrically determined in future
float k_phi = 0.7; //controls the servo turning angle, delta
float v_max = 5; //maximum safe velocity for car, [m/s]?
float c_accel = 0.4; //acceleration-to-motor-input conversion, experiments needed [s], positive
float k_delta = -v_max/90; // [(m/s)/degree] ...no penalty at zero turn angle, max penalty at 90 degree turn
float c_p = 3; //DC motor constant, must be experimentally determined, [(m/s)/(PWM%)]

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // minimum information only
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
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

}

SIGNAL(TIMER0_COMPA_vect) {
   char c = GPS.read();
}

void loop() {
  // put your main code here, to run repeatedly:
  int tau=4;
  analogWrite(motorPin,motorPWMCurrent); //motor PWM could be changing based on feedback
  static unsigned long timer = millis();
  
  //first part heading
  //newly added, read from map or image
  byte psimap = 50;
  byte betaimage = 20;
  byte lowFHeading = psimap + betaimage;  
  lsm.read();  /* ask it to read in the data */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  byte highFDotHeading = g.gyro.z;
  static float heading_est_degrees = 0;
  heading_est_degrees = (lowFHeading + tau * highFDotHeading - heading_est_degrees) / tau * (.001 * timeRead);
  
  //second part heading
  //newly added, read from map or image
  float widthmap = 2;
  float widthcamera = 0.2;
  float speed_estimate = 1; //how to derive this?
  float lowFOffset = widthmap / 2 -  widthcamera * sin(heading_est_degrees - psimap);
  float highFDotOffset = speed_estimate * sin(heading_est_degrees - psimap) + frontEndL * highFDotHeading * cos(heading_est_degrees - psimap);
  static float offset_est = 0;
  offset_est = (lowFOffset + tau * highFDotOffset - offset_est) / tau *(.001 * timeRead);
  
  //third part distance
  float currGPSLat = 0;
  float currGPSLon = 0;
  float startGPSLat = 39.95;
  float startGPSLon = 75.18;
  float highFDotDistance = speed_estimate * cos(heading_est_degrees - psimap) - frontEndL * highFDotHeading * sin(heading_est_degrees - psimap);
  if (GPS.newNMEAreceived()){
    if (GPS.parse(GPS.lastNMEA())){
      currGPSLat = GPS.latitudeDegrees;
      currGPSLon = GPS.longitudeDegrees;
    }
  }
  float lowFDistance = 1000 * distanceInKmBetweenEarthCoordinates(currGPSLat,currGPSLon,startGPSLat,startGPSLon);
  static float distance_est = 0;
  distance_est = (lowFDistance + tau * highFDotDistance - distance_est) / tau * (.001 * timeRead);
  //control time of loop
  while(millis()-timer<20){
  }
  timer=millis();

  //Control Loop <Tage>
  //convert width of road to prefered distance from center
  float offset_desired = k_width * widthmap; //is widthmap the width of road? //offset_desired is negative if right(CW) from road center
  float offset_Beta = offset_desired + offset_est;
  float beta_desired = offset_Beta * k_offset; //positive if going CCW
  float delta = servoAngle + k_phi*(psimap + beta_desired - heading_est_degrees);
  //speed control
  float vpenalty_accel = a.acceleration.z * -(c_accel);
  float vpenalty_delta = delta*k_delta;
  float v_desired = v_max + vpenalty_delta + vpenalty_accel;
  float motor_pwm = pwm0 + (v_desired/c_p);
  analogWrite(motorPin,motor_pwm);
}

float degreesToRadians(float degrees) {
  return degrees * Pi / 180;
}

float distanceInKmBetweenEarthCoordinates(float lat1, float lon1, float lat2, float lon2) {
  float earthRadiusKm = 6371;

  float dLat = degreesToRadians(lat2-lat1);
  float dLon = degreesToRadians(lon2-lon1);

  lat1 = degreesToRadians(lat1);
  lat2 = degreesToRadians(lat2);

  float a = sin(dLat/2) * sin(dLat/2) +
          sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2); 
  float c = 2 * atan2(sqrt(a), sqrt(1-a)); 
  return earthRadiusKm * c;
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
