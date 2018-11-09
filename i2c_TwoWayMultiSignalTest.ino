//
// some useful stuff
// http://gammon.com.au/i2c
//

#include <Wire.h>

#define SLAVE_ADDRESS 0x04

//
// global variables
// (yucky but needed to make i2c interrupts work)
//
byte piCommand;
String value;
String offset;

void setup() {
//
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveDataI2C);
    Wire.onRequest(sendDataI2C);

    Serial.begin(115200);
}


//////////////////////////////////////////////////////////////////
void loop() {
  delay(500);  // update every half second

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
         float finalValue = tempString.toFloat();
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
         float finalValue = tempString.toFloat();
         Serial.println(finalValue);
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
