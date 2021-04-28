#include <Wire.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(24);
  Wire.onReceive(myHandler);
  Wire.onRequest(requestEvent); // register event
  delay(100);
  Serial.println("Ausgabe von I2C Daten an Adresse 24:");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}
void myHandler(int NumBytes){
  // loop through all bytes 3 times
  byte buff[NumBytes];
  for (int i=0; i<NumBytes; i++){
    // read in Bytes
    buff[i] = Wire.read(); 
  }

  Serial.print("Chars: ");
  Serial.print((char*)buff);

  Serial.println();
  
  for (int i=0; i<NumBytes; i++){
    // print out Bytes as number
    Serial.println(buff[i]);
  }
  Serial.println();
  Serial.print("Nachrichtenlänge: ");Serial.print(NumBytes);
  if(NumBytes==8){
    Serial.println();
    Serial.print("Fahrbefehle: ");
    
    union{
      byte by[8];
      struct  {
        int velocity;
        int dist ;
        int angle;
        byte prio;
        byte type;
      } einzeln; 
    }fahrbefehle;
    
    for(int i=0; i<8; i++){
      fahrbefehle.by[i]=buff[i];
    }
    
    Serial.print("Geschw.: "); Serial.print(fahrbefehle.einzeln.velocity);Serial.print("Distanz: "); Serial.print(fahrbefehle.einzeln.dist);
    Serial.print("Winkel: "); Serial.print(fahrbefehle.einzeln.angle);Serial.print("Prioriät: "); Serial.print(fahrbefehle.einzeln.prio);
    Serial.print("Typ: "); Serial.print(fahrbefehle.einzeln.type);

  }
  
  
}

void requestEvent(){
  Wire.write("Response from Device No. 24");
}

