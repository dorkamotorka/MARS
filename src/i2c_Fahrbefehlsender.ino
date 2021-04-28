#include <Wire.h>
#include <Adafruit_MotorShield.h>


//Union um Daten von der I2C-Schnittstelle zu Speichern und in Form von Longs zugänglich zu machen
union achtByteNachricht {
  byte by[8];              //Befehl ist 8 Byte lang (s. Inhalt vom folgenden Struct)
  struct  {                 //Struct zum Ordnen der Daten
    int vsollneu;          //Sollgeschwindigkeit des Fahrzeugmittelpunktes des gesendeten Befehls [in 1 mm pro Sekunde]
    int ssollneu ;         //Sollweg des Fahrzeugmittelpunktes des gesendeten Befehls [in 1 mm]
    int phisollneu;         //Solldrehwinkel des gesendeten Befehls [in Grad] Rechtskurve sind positive Drehwinkel bei vorwärtsfahrt
    byte priosollneu;       //Priorität  des gesendeten Befehls(255 = Sofortbefehl)(1 geringste Priorität)
    byte typsollneu;        //Beschreibt den Typ des gesendeten Befehls; (Wenn grader Wert, dann werden alle anderen Werte nach hinten geschoben, wenn ungerade, dann werden alle folgenden Werte gelöscht)
  } Fahrbefehl;        //Struct im Union heißt Fahrbefehl
} Befehle;



void setup()
{

  Wire.begin();        // join i2c bus (address optional for master)
  delay(3000);

}

int vsoll[5] = {0, 300, -300, 300, 300};
int ssoll[5] = {0, 100, -100, 100, 100};
int phisoll[5] = {0, 0, 0, 30, -30}; //Rechtskurve sind positive Drehwinkel
byte priosoll[5] = {255, 255, 255, 255, 255};
byte typsoll[5] = {1, 1, 1, 1, 1};
int i = 0;




void loop() {


  Befehle.Fahrbefehl.vsollneu = vsoll[i];
  Befehle.Fahrbefehl.ssollneu = ssoll[i];
  Befehle.Fahrbefehl.phisollneu = phisoll[i];
  Befehle.Fahrbefehl.priosollneu = priosoll[i];
  Befehle.Fahrbefehl.typsollneu = typsoll[i];
  i = i + 1;
  if (i == 5) {
    i = 0;
  }
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

  delay(4000);


}


boolean I2Creservieren () {
  while (1) {
    if (analogRead(A2) <= 200) {              //Checks the voltage of the busyline
      pinMode(A2, INPUT_PULLUP);                 //Activates the internal pull-up resistor(30k - 33k Ohm)
      delayMicroseconds(100);                    //Waits until the busyline adapted the new voltage
      if (analogRead(A2) <= 540) {               //Checks the busyline again
        return (1);
      }                            //The function returns 1 after a successful transmission
      else {                                     //In case two masters tried to reserve the bus
        pinMode(A2, INPUT);                     //Deactivates the internal pull-up resistor
        delayMicroseconds(random(100, 1000));
      }
    } //Waits a random short time
    else {                                       //In case the Busyline was reserve from the beginning
      pinMode(A2, INPUT);
    }                     //Deactivates the internal pull-up resistor
  }
}
boolean I2Cfreigeben() {
  pinMode(A2, INPUT);
  return (1);
}



