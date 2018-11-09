#include <Servo.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x04
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
byte motorPWMCurrent=0; //useful for static-speed tests
float pingDistanceCM = 0; //m
float timeRead=20; //[ms]
float gpsPsi;
float frontEndL = 0.15; //m
float Pi = 3.1416;
float widthcamera = 0.2; //units???
  byte betaimage = 20;
//float headingconstant = 0.1
float heading_est_degrees = 0;
float offset_est = 0;

//control constants
float k_width = 0.2;
float k_offset = 50; //converts offset change into an angle in degrees, geometrically determined
float k_phi = 0.7; //controls the servo turning angle, delta

//motor contants
float pwm0 = 45/255; //experimentally determine pwm0 for DC 
float v_max = 3; //maximum safe velocity for car, [m/s]
float c_accel = 0.4; //acceleration-to-PWM conversion, experiments needed [s], positive
float c_p = 1/70; //DC motor constant, must be experimentally determined, [(m/s)/(PWM%)]
float k_delta = -v_max/90; // [(m/s)/degree] ...no penalty at zero turn angle, max penalty at 90 degree turn
float speed_estimate = 0; //how to derive this?

//I2C
byte piCommand;
String value;
String offset;

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
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveDataI2C);
    Wire.onRequest(sendDataI2C);

    Serial.begin(115200);
}

SIGNAL(TIMER0_COMPA_vect) {
   char c = GPS.read();
}

void loop() {
  int tau=4;
  static unsigned long timer = millis();
  
  //first part heading
  //newly added, read from map or image
  byte psimap = 0;

  byte lowFHeading = psimap + betaimage;  
  lsm.read();  /* ask it to read in the data */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  byte highFDotHeading = g.gyro.z;
  heading_est_degrees = (lowFHeading + tau * highFDotHeading - heading_est_degrees) / tau * (.001 * timeRead);

  //1.5th part: Speed
  //filter GPS data (Low-Pass) + maybe use pixel distance change from camera (high-pass)
  
  //second part offset from road
  //newly added, read from map or image
  float widthmap = 2; //units???
  float lowFOffset = widthmap / 2 -  widthcamera * sin(heading_est_degrees - psimap);
  float highFDotOffset = speed_estimate * sin(heading_est_degrees - psimap) + frontEndL * highFDotHeading * cos(heading_est_degrees - psimap); //from Bruce's slides
  offset_est = (lowFOffset + tau * highFDotOffset - offset_est) / tau *(.001 * timeRead);
  
  //third part distance
  float currGPSLat = 0;
  float currGPSLon = 0;
  float startGPSLat = 39.95; //hard-coded, will change to a GPSread
  float startGPSLon = 75.18; //hard-coded, will change to a GPSread
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

  //Control Loop
  //convert width of road to prefered distance from center
  float offset_desired = k_width * widthmap; //is widthmap the width of road? //offset_desired is negative if right(CW) from road center
  float offset_Beta = offset_desired + offset_est;
  float beta_desired = offset_Beta * k_offset; //positive if going CCW
  float delta = servoAngleDeg + k_phi*(psimap + beta_desired - heading_est_degrees);
  steeringServo.write(constrain(delta,60,120));
  
  //speed control
  float vpenalty_accel = a.acceleration.z * -(c_accel);
  float vpenalty_delta = delta*k_delta;
  float v_desired = v_max + vpenalty_delta + vpenalty_accel;
  float speed_estimate = v_desired; //how to derive this?
  float motor_pwm = 255*(pwm0 + (v_desired/c_p));
  //check for obstacles
  getPingDistanceCM();
  if(pingDistanceCM<=30.0){
    motor_pwm=0;
  } 
  //finally, write command to motor
  analogWrite(motorPin,motor_pwm);
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
        dataBuffer[0] = heading_est_degrees;//estHeading;
        dataBuffer[1] = offset_est;//estOffset;
        dataBuffer[2] = 0;//estSpeed
        Wire.write((byte*) &dataBuffer[0], 3*sizeof(float));
    }

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
