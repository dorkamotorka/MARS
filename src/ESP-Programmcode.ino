    /******************************************************************************
     *       Arduino Program for the ESP8266-Module in the MARS Field-Robot       *
     *             by Jan Beckmann (jan.beckmann@rwth-aachen.de)                  *
     ******************************************************************************/
  
#include <ESP8266WiFi.h>    // includes library with functions for the TCP Server
#include <Wire.h>           // inlcudes library for I2C-Connection
#include <Adafruit_PWMServoDriver.h>  //includes libraray for the rear power lifter

/*    <-- Some information about the I2C Connection -->
 *    There are both 7- and 8-bit versions of I2C addresses. 
 *    7 bits identify the device, and the eighth bit determines if it's being written to or read from. 
 *    The Wire library uses 7 bit addresses throughout. 
 *    If you have a datasheet or sample code that uses 8 bit address, you'll want to drop the low bit (i.e. shift the value one bit to the right), yielding an address between 0 and 127. 
 *    However the addresses from 0 to 7 are not used because are reserved so the first address that can be used is 8. 
 *    Please note that a pull-up resistor is needed when connecting SDA/SCL pins. Please refer to the examples for more informations. 
 *    MEGA 2560 board has pull-up resistors on pins 20 - 21 onboard.
 *    The Wire library implementation uses a 32 byte buffer, therefore any communication should be within this limit. 
 *     Exceeding bytes in a single transmission will just be dropped.
 */
 
#define MAX_CLIENTS 5                       // More than one client is possible, every Screen is an own Client
#define TCP_PORT (23)                       // Pre-defined in Android-APP
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpServerClients[MAX_CLIENTS];

IPAddress apIP(192, 168, 1, 1);            //Pre-defined in Android-APP
const char SSID[] = "MARSRoboter";         // Wifi-Name
const char PASSWORD[] = "12345678";        // Passwort

Adafruit_PWMServoDriver I2Cpwm = Adafruit_PWMServoDriver(0x41);  //create the I2Cpwm object for rear power lift
byte depthRearPowerLifter = 0;                                       
boolean rearPowerSet= false;     //to only send above variable when its set

//  ***********************************
//  *   given I2C-master-functions    *
//  ***********************************

boolean I2Creservieren () { 
  /*
//Function to reserve the bus
//Checks the voltage of the busyline with A0
//Activates the internal pull-up resistor(10k) of D3
//Waits until the busyline adapted the new voltage 
//Checks the busyline again
//The function returns 1 after a successful transmission
//In case two masters tried to reserve the bus 
//Deactivates the internal pull-up resistor //Waits a random short time
//In case the Busyline was reserve from the
//Deactivates the internal pull-up resistor //Function to release the bus
 */
  while (1) {
    if (analogRead(A0) <= 200) {
      pinMode(D5, OUTPUT);
      digitalWrite(D5, HIGH);
        //Serial.println("I2C reserviert");     //Line for debugging
      delayMicroseconds(100);
      if (analogRead(A0) <= 525){
          return (1);
          
      }else{
        pinMode(D5, INPUT); 
        delayMicroseconds(random(100, 1000));}
    }
    else{
        pinMode(D5, INPUT);
      }
  }
}

boolean I2Cfreigeben() {
  //Serial.println(" I2C freigegeben");         //Line for debugging
  pinMode(D5, INPUT);
  return (1);
}

//  ***********************************
//  *      Setup and Main Loop        *
//  ***********************************

void setup() {
  pinMode (D5, Input);                    //Busyline should be free at start-Up
  Serial.begin(9600);                     // for debugging on serial monitor in Arduino IDE
  delay(100);
  Wire.begin();                           //join I²C as Master
  delay(100);
  WiFi.mode(WIFI_AP);                      //Set up Access Point
  //  .softAPConfig (local_ip, gateway, subnet)
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  //   .softAP(ssid, password, channel, hidden, max_connection)
  WiFi.softAP(SSID, PASSWORD);
  
  tcpServer.begin();                      //set up TCP-Server for APP-Connection
  tcpServer.setNoDelay(true);
  
  I2Creservieren();                       //initialize rear power lifter
  I2Cpwm.begin();
  I2Cpwm.setPWMFreq(50);
  I2Cfreigeben();
  
  delay(1000);                           //print some Information to Serial Monitor:
  Serial.println("****************************");
  Serial.println("* MARS-ESP8266 Version 0.5 *");
  Serial.println("****************************");
  Serial.println();
  Serial.print("Ready! IP-Adress is: ");
  Serial.print(WiFi.localIP());
  Serial.print(" Port: ");
  Serial.println(TCP_PORT);
  Serial.println("Last reset Reason: ");
  Serial.println(ESP.getResetReason());
  
  /* some testcode to run at startup:
    Wire.beginTransmission(10); 
    Wire.write("Test der I2C Verbindung"); 
    Wire.endTransmission(); // stop transmitting
  */
}

void loop() {
  uint8_t i;
  char buf[325]; //Buffer for 324 chars + null character (32 Byte : 256 Bit + max. Lenght of Typelist (32 Types +" ") + Brackets
  int bytesAvail, bytesIn;

  memset(buf, 0, sizeof(buf));                 // clear  Buffer (rewrite Buffer with zeros)
  
  //check if there are any new clients
  if (tcpServer.hasClient()) {
    for (i = 0; i < MAX_CLIENTS; i++) {
      //find free/disconnected spot
      if (!tcpServerClients[i] || !tcpServerClients[i].connected()) {
        if (tcpServerClients[i]) tcpServerClients[i].stop();
        tcpServerClients[i] = tcpServer.available();
        //Serial.print("New client: "); Serial.print(i);                  //Line for debugging
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient tcpServerClient = tcpServer.available();
    tcpServerClient.stop();
  }

  //check clients for data
  for (i = 0; i < MAX_CLIENTS; i++) {
    if (tcpServerClients[i] && tcpServerClients[i].connected()) {
      while ((bytesAvail = tcpServerClients[i].available()) > 0) {
        bytesIn = tcpServerClients[i].readBytes(buf, bytesAvail);   //read Data from the APP via TCP-Connection 
        if (bytesIn > 0) {

         //incoming data format (example): ((2 4 3 6 1 ) 50 50 36598 25 abc 25 )
         
          String dataIn = String(buf);
          String typeList = dataIn.substring(2,dataIn.indexOf(')'));
          String dataList = dataIn.substring(dataIn.indexOf(')')+2,dataIn.length()-1);
          /* when using the Connection without Filter, only the first message will be used by ESP-Module
          if(dataList.indexOf(')')!=-1){              // simple Check for right format, prevent messages, that doubled through quick sending or connection-failure
            Serial.print("Fehler bei Übertragung");   //Line for debugging
            break;
          }
          */
          int dataIndex=0;
          byte firstByte=(byte)dataList.substring(0, dataIn.indexOf(" ")).toInt();
          dataIndex=dataIn.indexOf(" ");
          
          /* Debugging Textlines:
          Serial.print("typeList: ");
          Serial.print(typeList);
          Serial.println();
          Serial.print("Data: ");
          Serial.print(dataList);
          Serial.println();
          */
          
          if(bitRead(firstByte, 0)==1){   //If I2C-Data should be requested
            // Serial.print("Anfrage an:"); Serial.print((firstByte>>1)); //Line for debugging
            
            I2Creservieren();
            
            if(Wire.requestFrom((firstByte>>1), dataList.substring(dataIndex, dataList.indexOf(' ',dataIndex+1)).toInt())){ //if Request succeeeded
              String slaveAnswerString="";
              slaveAnswerString +=(firstByte>>1);
              slaveAnswerString += ";";
              while(Wire.available()>0){    // slave may send less than requested
               
                slaveAnswerString += (int)Wire.read();    // receive a byte as number and store it, sperated by semicolons
                slaveAnswerString += ";";
              }
              I2Cfreigeben();
              
              /*
               Send Requested Data to APP:
               Don't loop over every Client, because just the one that asked needs the data (and is sure connected)
               Trying to iterate over all connected clients will cause an Error (Hardware Watchdog)
               */
               
              tcpServerClients[i].print(slaveAnswerString);
              delay(1);                                                    //do NOT remove
              
              /* Following commands are not sending to APP, but should send to all clients instead of just one:
              tcpServer.write(slaveAnswerCharArray);
              tcpServer.println(slaveAnswerString);
              It couldn't be figured out why they do not work.
              */
              
              //comment following out:
              Serial.println();
              Serial.print("Antwort des Slaves: ");
              Serial.print(slaveAnswerString);
              
            }
            else{
              I2Cfreigeben();
              Serial.println("Request not successful"); //Line for debugging
              break;                                    
            }
            
          }
          else{
            // Daten senden:
            I2Creservieren();
            Wire.beginTransmission(firstByte>>1); // transmit to device, Adress declared in first byte (Adress is 7 Bit not 8 Bit -> filter beforehand as described in Line 11)
            /*
             First read typeList to know array size, then iterate to fill a data array,
             or:
             Loop between wire commands and send individually
            */
            Serial.print("Wire sends to ");Serial.print(firstByte>>1);Serial.print(": ");
            for(int j=0; j<typeList.length(); j++){
              if(typeList.charAt(j)!=' '){
                if((typeList.charAt(j)=='1')||(typeList.charAt(j)=='2')){ //1 or 2 means Byte
                  Wire.write((byte)dataList.substring(dataIndex, dataList.indexOf(' ',dataIndex+1)).toInt());
                 Serial.print("Byte: "); Serial.println((byte)dataList.substring(dataIndex, dataList.indexOf(' ',dataIndex+1)).toInt());
                }
                else if(typeList.charAt(j)=='3'){ //3 is for an Int
                  union {                    //Build a union to send a Float as ByteArray, since Wire.write(Float) doesnt exist
                    byte byInt[4];
                    long helpInt;
                  } intUnion;
                  intUnion.helpInt=(int(dataList.substring(dataIndex, dataList.indexOf(' ',dataIndex+1)).toInt()));  //toInt() returns long, is typecast necessary?
                  Serial.print("Int: "); Serial.println(intUnion.helpInt);
                  Wire.write(intUnion.byInt, 2);
                }
                else if(typeList.charAt(j)=='4'){ //4 is for an Long
                  union {                    //Build a union to send a Float as ByteArray, since Wire.write(Float) doesnt exist
                    byte byLong[4];
                    long helpLong;
                  } longUnion;
                  longUnion.helpLong=(dataList.substring(dataIndex, dataList.indexOf(' ',dataIndex+1))).toInt();
                  Serial.print("Long: ");Serial.println(longUnion.helpLong);
                  Wire.write(longUnion.byLong, 4);
                  
                }
                else if(typeList.charAt(j)=='5'){ //5 is for an Float
                  union {                    //Build a union to send a Float as ByteArray, since Wire.write(Float) doesnt exist
                    byte byFloat[4];
                    float helpFloat;
                  } floatUnion;
                  floatUnion.helpFloat=(dataList.substring(dataIndex, dataList.indexOf(' ',dataIndex+1))).toFloat();
                  Serial.print("Float: ");Serial.println(floatUnion.helpFloat);
                  Wire.write(floatUnion.byFloat, 4);
                }
                else if(typeList.charAt(j)=='6'){ //6 is for an String
                  char stringBuf[33];         //altough Wire.write(string) is mentioned in the Reference, it actually needs a Char-Array
                  (dataList.substring(dataIndex, dataList.indexOf(' ',dataIndex+1))).toCharArray(stringBuf, 32); //convert String to Char Array with Buffer of Size 32 + null character, wich should be enough for I2C Messages
                  Wire.write(stringBuf);
                  Serial.println(stringBuf);
                }
                else if(typeList.charAt(j)=='7'){ //7 is for an I2C PWM-Command (right now only for rear power lifter
                  depthRearPowerLifter = ((byte)dataList.substring(dataIndex, dataList.indexOf(' ',dataIndex+1)).toInt());
                  rearPowerSet=true; //send only when changed
                  //set rear power lift after Wire.endTrasmission() but before releasing the busy line
                  Serial.print("Höhe rear-power lifter: ");
                  Serial.print((byte)dataList.substring(dataIndex, dataList.indexOf(' ',dataIndex+1)).toInt());
                  Serial.println();
                }
                else{
                  Serial.print("typeList Error");
                }
                //Serial.print("Data Index alt: "); Serial.print(dataIndex);Serial.print(" Char: ");Serial.print(dataList.charAt(dataIndex));   //Line for debugging
                dataIndex=dataList.indexOf(' ',dataIndex+1);
               // Serial.print("  Data Index neu: "); Serial.print(dataIndex);Serial.print(" Char: ");Serial.print(dataList.charAt(dataIndex)); //Line for debugging
               //Serial.println();
              }
            }
            Wire.endTransmission(); // stop transmitting
            
            if(rearPowerSet){
              I2Cpwm.setPWM(0, 0, map(depthRearPowerLifter, 0, 57, 170, 510)); //the rear power lifter is actuated by RC Servo via an PCA9685 I2C to PWM converter
              rearPowerSet=false;
            }
            I2Cfreigeben();
          }
        
          Serial.println();
        }
      }
    }
  }
}

