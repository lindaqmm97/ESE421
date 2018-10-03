float gpsPsi;
float servoAngle=90;
int tau=4;

double desiredPsi=10;
if (GPS.parse(GPS.lastNMEA())){
    if(GPS.speed>0.5){
       gpsPsi = GPS.angle;}
    }
sensors_event_t a, m, g, temp;
lsm.getEvent(&a, &m, &g, &temp);
static float heading_est_degrees = 0;
float partial_sub=gpsPsi-heading_est_degrees;
Serial.println(partial_sub);
if(partial_sub<-180){
   partial_sub+=360;
}
else if(partial_sub>180){
   partial_sub-=360;
}
heading_est_degrees+=(.001*timeRead)/tau*(gpsPsi-heading_est_degrees)+(.001*timeRead)*g.gyro.z;
float angle_sub=desiredPsi-heading_est_degrees;
if(desiredPsi-heading_est_degrees<-180){
  angle_sub=360+angle_sub;
}
else if(desiredPsi-heading_est_degrees>180){
  angle_sub=angle_sub-360;
}  
servoAngle = servoAngleDeg-k_heading * angle_sub; 

steeringServo.write(constrain(servoAngle,servoAngleDeg-30, servoAngleDeg+30)); 
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
