//Payload Flight Software 2023 CanSat by Max Epstein, Yashas Shivaram, and Dylan Falzone
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

float seaLevel;

#define TEAM_ID 1032;
uint16_t PACKET_COUNT = 0;
char MISSION_TIME[7] = "00:00:00";
uint8_t MODE = 70;
uint8_t STATE = 'S';
float ALTITUDE = 0.00;
uint8_t HS_DEPLOYED = 'N';
uint8_t PC_DEPLOYED = 'N';
uint8_t MAST_RAISED = 'N';
String GPS_TIME = "00:00:00";
String CMD_ECHO = "";
int n = 0;

char radioSend[110]; // Character array to store the compiled data
char buffer[40]; // Temporary buffer for converting values to strings


float apogee = 670;
boolean camRec = false;
boolean simAct = false;
boolean sendTelemetry = false;

// Create objects for sensors
Adafruit_BMP280 bmp; // I2C
SoftwareSerial USART2(9,10); //USART for radio
#define GPSSerial Serial
Adafruit_GPS GPS(&GPSSerial); // UART
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  //Serial.begin(115200);
  GPS.begin(9600);
  USART2.begin(19200);
  bmp.begin();
  bno.begin();


  // Initialize Adafruit Ultimate GPS
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  //camera 
  pinMode(4, OUTPUT); // Configure pin 4 as an Output
  digitalWrite(4, HIGH); // Initialize pin 4 as High

  //Servo 
  pinMode(8, OUTPUT); // Configure pin 5 as an Output
  
  while(n < 355){ //must be 355
    digitalWrite(8, HIGH);    // Set pin D5 HIGH
    delayMicroseconds(400);  // Wait for 1ms
    
    digitalWrite(8, LOW);     // Set pin D5 LOW
    delayMicroseconds(19600); // Wait for 19ms
    n++;
  }
  

  //Sound Beacon 
  pinMode(6, OUTPUT); // pin 6 is speaker positive
  digitalWrite(5, LOW); 
  pinMode(5, OUTPUT); //pin 5 is speaker ground
  digitalWrite(6, LOW); 


}
void loop() {
      
  //get new incomming
  if (USART2.available() > 0){
    CMD_ECHO =USART2.readString();
    //Serial.println(CMD_ECHO);
    CMD_ECHO = CMD_ECHO.substring(9);


        
      if (CMD_ECHO[2] == 'L') {
        seaLevel = bmp.readAltitude(1013.25);
        STATE = 'S';
      }
      else {
        switch (CMD_ECHO[4]) {
          case 'N':
            sendTelemetry = true;
            break;
          case 'F':
            sendTelemetry = false;
            break;
          case 'E':
            //MODE = 'S';
            break;
          case 'A':
            simAct = true;
            break;
          case 'D':
            //MODE = 'F';
            simAct = false;
            break;
          default:
            ALTITUDE = bmp.readAltitude(CMD_ECHO.substring(4).toFloat());
            break;
        }
      }
      
      if(simAct == true){
        MODE = 83;
      }
      else{
        MODE = 70;
      }


  

      if (CMD_ECHO[1] == 'X'){
        CMD_ECHO.remove(2,1);
      }
      else {
        CMD_ECHO.remove(3,1);
      }

  }//end of if avaiable statement
  
  char c = GPS.read();
  
    if (GPS.newNMEAreceived()) {
  
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }

  if (millis()/1000 - PACKET_COUNT > 1 && sendTelemetry && millis()%1000 == 0) {
    PACKET_COUNT += 1; //increase the packet count by 1

    if (simAct == false){
      ALTITUDE = (bmp.readAltitude(1013.25)-seaLevel);
    }
    if (ALTITUDE > apogee){
      apogee = ALTITUDE;
    }

    uint32_t runtime = millis();
    MISSION_TIME[0] = ((runtime/60000)%60)/10 +'0'; //minutes high
    MISSION_TIME[1] = ((runtime/60000)%60)%10 +'0'; //minutes low
    MISSION_TIME[3] = ((runtime/1000)%60)/10 + '0'; //seconds high
    MISSION_TIME[4] = ((runtime/1000)%60)%10 + '0'; //seconds low
    MISSION_TIME[6] = ((runtime/3600000)%100)/10 +'0'; //cs high
    MISSION_TIME[7] = ((runtime/3600000)%100)%10 +'0'; //cs low
    MISSION_TIME[8] = '\0';

    //DO ALL LOGIC HERE
    if (STATE == 'S' && PACKET_COUNT > 1){
      STATE = 'W';
    }
    else if (ALTITUDE > 60 && STATE == 'W'){
      STATE = 'A';
    }
    else if(apogee <= ALTITUDE){
      STATE = 'R';
    }
    else if (ALTITUDE < 500 && STATE == 'R'){//payload is released from can, heatshield deployed
      STATE = 'P' ;   
      HS_DEPLOYED = 'P';
      //start recording
      digitalWrite(4, LOW);
      delay(1000);
      digitalWrite(4,HIGH);
    }
    else if (ALTITUDE < 200 && STATE == 'P'){//DESCENT
      STATE = 'D';
      //parachute:
      PC_DEPLOYED = 'C';
      n = 0;
      while(n < 355){ //must be 355
      digitalWrite(8, HIGH);    // Set pin D5 HIGH
      delayMicroseconds(2500);  // Wait for 1ms
      
      digitalWrite(8, LOW);     // Set pin D5 LOW
      delayMicroseconds(20000 - 2500); // Wait for 19ms
      n++;
      }
      //turn recording back on (servo turned it off)
      digitalWrite(4, LOW);
      delay(1000);
      digitalWrite(4,HIGH);
    }
    else if(ALTITUDE < 60 && STATE == 'D'){//LANDED
      STATE = 'L';
      //end recording
      digitalWrite(4, LOW); 
      delay(1000);
      digitalWrite(4,HIGH);

      //Audio beacon:
      tone(6, 700);  // Change the frequency as desired
    }
    

    
    GPS_TIME[0] = GPS.hour/10 + '0';
    GPS_TIME[1] = GPS.hour%10 + '0';
    GPS_TIME[3] = GPS.minute/10 + '0';
    GPS_TIME[4] = GPS.minute%10 + '0';
    GPS_TIME[6] = GPS.seconds/10 + '0';
    GPS_TIME[7] = GPS.seconds%10 + '0';
    
    // Read and print data from BNO055
    sensors_event_t event;
    bno.getEvent(&event);   

    
    strcpy(radioSend, "1032");
    strcat(radioSend, ",");
    //strcat(radioSend, MISSION_TIME.c_str());
    strcat(radioSend, MISSION_TIME);
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(PACKET_COUNT, 1, 0, buffer));
    strcat(radioSend, ",");
    buffer[0] = MODE; // Convert STATE to a character
    buffer[1] = '\0'; // Null-terminate the buffer
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    buffer[0] = STATE; // Convert STATE to a character
    buffer[1] = '\0'; // Null-terminate the buffer
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(ALTITUDE, 0, 2, buffer));
    strcat(radioSend, ",");
    buffer[0] = HS_DEPLOYED; // Convert STATE to a character
    buffer[1] = '\0'; // Null-terminate the buffer
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    buffer[0] = PC_DEPLOYED; // Convert STATE to a character
    buffer[1] = '\0'; // Null-terminate the buffer
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    buffer[0] = MAST_RAISED; // Convert STATE to a character
    buffer[1] = '\0'; // Null-terminate the buffer
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(bmp.readTemperature(), 0, 2, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(bmp.readPressure() / 100.0F, 0, 2, buffer));
    strcat(radioSend, ",");

    //****************************************************
    //CHANGE A0 to whatever pin is connected to battery
    //****************************************************
    strcat(radioSend, dtostrf((analogRead(A0) * 5.0) / 1024.0, 0, 2, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, GPS_TIME.c_str());
    //strcat(radioSend, GPS_TIME);
    strcat(radioSend, ",");
    dtostrf(GPS.latitude / 100.0, 7, 4, buffer);
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    dtostrf(GPS.longitude / 100.0, 7, 4, buffer);
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(GPS.altitude, 0, 0, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(GPS.satellites, 0, 0, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(event.orientation.x, 0, 2, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(event.orientation.y, 0, 2, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, CMD_ECHO.c_str());

    if (sendTelemetry == true){
      USART2.println(radioSend); 
    }
  }
}
