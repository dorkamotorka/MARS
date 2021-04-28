/*

   Stand 1.6.2018 getestet. Funktioniert.
*/

#include <Wire.h>


#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver I2Cpwm = Adafruit_PWMServoDriver(0x41);


//Union um Daten von der I2C-Schnittstelle zu Speichern und in Form von Longs zugänglich zu machen
union achtByteNachricht {
  byte by[8];              //Befehl ist 12 Byte lang (s. Inhalt vom folgenden Struct)
  struct  {                 //Struct zum Ordnen der Daten
    int vsollneu;          //Sollgeschwindigkeit des Fahrzeugmittelpunktes des gesendeten Befehls [in 1 mm pro Sekunde]
    int ssollneu ;         //Sollweg des Fahrzeugmittelpunktes des gesendeten Befehls [in 1 mm]
    int phisollneu;         //Solldrehwinkel des gesendeten Befehls [in Grad] Rechtskurve sind positive Drehwinkel bei vorwärtsfahrt
    byte priosollneu;       //Priorität  des gesendeten Befehls(255 = Sofortbefehl)(1 geringste Priorität)
    byte typsollneu;        //Beschreibt den Typ des gesendeten Befehls; (Wenn grader Wert, dann werden alle anderen Werte nach hinten geschoben, wenn ungerade, dann werden alle folgenden Werte gelöscht)
  } Fahrbefehl;        //Struct im Union heißt Fahrbefehl
} Befehle;                  // Union heißt Befehle



void setup()
{
  delay(2000);
  Wire.begin(52); //Hauptcontroller hat die Adresse 8 dies ist auch das Führugnsbyte
  Serial.begin(9600);

  pinMode(A2, INPUT);

  I2Creservieren();
  I2Cpwm.begin();

  I2Cpwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
  //delay(100);
  byte tiefe = 20;
  I2Cpwm.setPWM(0, 0, map(tiefe, 0, 57, 170, 510));

  I2Cfreigeben();

}




void loop() {
  delay(2000);
  I2Creservieren();
  byte tiefe = 50;
  I2Cpwm.setPWM(0, 0, map(tiefe, 0, 57, 170, 510));
  I2Cfreigeben();


  delay(2000);
  Befehle.Fahrbefehl.vsollneu = 50;
  Befehle.Fahrbefehl.ssollneu = 50;
  Befehle.Fahrbefehl.phisollneu = 0;
  Befehle.Fahrbefehl.priosollneu = 2;
  Befehle.Fahrbefehl.typsollneu = 0;
  I2Creservieren();
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(Befehle.by[0]);
  Wire.write(Befehle.by[1]);
  Wire.write(Befehle.by[2]);
  Wire.write(Befehle.by[3]);
  Wire.write(Befehle.by[4]);
  Wire.write(Befehle.by[5]);
  Wire.write(Befehle.by[6]);
  Wire.write(Befehle.by[7]);
  Wire.endTransmission();    // stop transmitting
  I2Cfreigeben();


  delay(2000);
  I2Creservieren();
  tiefe = 20;
  I2Cpwm.setPWM(0, 0, map(tiefe, 0, 57, 170, 510));
  I2Cfreigeben();


  delay(2000);
  Befehle.Fahrbefehl.vsollneu = -50;
  Befehle.Fahrbefehl.ssollneu = -50;
  Befehle.Fahrbefehl.phisollneu = 0;
  Befehle.Fahrbefehl.priosollneu = 2;
  Befehle.Fahrbefehl.typsollneu = 0;
  I2Creservieren();
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(Befehle.by[0]);
  Wire.write(Befehle.by[1]);
  Wire.write(Befehle.by[2]);
  Wire.write(Befehle.by[3]);
  Wire.write(Befehle.by[4]);
  Wire.write(Befehle.by[5]);
  Wire.write(Befehle.by[6]);
  Wire.write(Befehle.by[7]);
  Wire.endTransmission();    // stop transmitting
  I2Cfreigeben();


}


boolean I2Creservieren () {
  while (1) {
    if (analogRead(A2) <= 200) {                 //Checks the voltage of the busyline
      pinMode(A2, INPUT_PULLUP);                 //Activates the internal pull-up resistor(30k - 33k Ohm)
      delayMicroseconds(100);                    //Waits until the busyline adapted the new voltage
      if (analogRead(A2) <= 540) {               //Checks the busyline again
        return (1);
      }                                          //The function returns 1 after a successful transmission
      else {                                     //In case two masters tried to reserve the bus
        pinMode(A2, INPUT);                      //Deactivates the internal pull-up resistor
        delayMicroseconds(random(100, 1000));
      }
    }                                            //Waits a random short time
    else {                                       //In case the Busyline was reserve from the beginning
      pinMode(A2, INPUT);
    }                     //Deactivates the internal pull-up resistor
  }
}


boolean I2Cfreigeben() {

  pinMode(A2, INPUT);
  return (1);
}
