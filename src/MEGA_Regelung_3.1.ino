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


//Speicher für die Befehle (Beschreibung s. union Befehle)(Der erste Befehl (Arrayplatz=0) wird zuerst verarbeitet)
int vsoll[20] = {100, 100 , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int ssoll[20] = {30, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int phisoll[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte priosoll[20] = {6, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte typsoll[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//Komtainer für den I2c Empfang
union I2Ckontainer {
  byte vByte[4];
  struct {
    int vRechts;
    int vLinks;
  } vSammlung;
} vWerte;


double Spurweite = 415; //Eigentlich 284 aber der Wert hat durch Schlupf eher den Solldrehwinkel wergeben [in 1/10 mm]


int Position = 0; //Position im Fahrbefehl Array an dem ein neu angekommener Befehl in Abhängigkeit seiner Priorität eingeordnet wird.
long Zeitbedarf = 0; //Berechnete Zeit die ein Fahrbefehl dauert. Anschließend kommt der nächste.
long dauer = 0;   //Variable für den Zeitbedart in der vregler Funktion (kann ggf. auch dorthin verschoben werden)

long letztesMal = 0; //Zeit in ms wann der letzte Fahrbefehl aufgeführt wurde


double vInnen = 0; //Geschwindigkeit des Kurveninneren Rades
double vAussen = 0;//Geschwindigkeit des Kurvenäußeren Rades
double sRadius = 0;//Radius der Kurvenfahrt (Fahrzeugmittelpunkt)
double sInnen = 0;//
double sAussen = 0;

int fertig = 0;
int neuerBefehl = 0;
int SOC = 0;
long letztesMalSOC = 0;
void setup()
{
  Wire.begin(8); //Hauptcontroller hat die Adresse 8 dies ist auch das Führugnsbyte
  Serial.begin(9600);
  //delay(100);
  //Serial.println("Gestartet");

  SOC = constrain(map(analogRead(7), 711, 856, 0, 100), 0, 100); // Fünf Zellen, aber mit 1/4,9 Spannungsteiler (1k:3,9k) liest man über den ADC die Einzelzellspannung aus. Voll 4,1v leer 3.4V. Somit gesamtpack: Voll 20,5 und leer 17V. Bei 1k:3,9k ergeben sich Spannunnge von 4,18V und 3,47. Bezogen auf 5V Referenz ergeben sich ADC 10bit ADC Werte von 856 und 711.
  //Serial.print("SOC:");
  //Serial.println(SOC);
  //delay(100);
  pinMode(A2, INPUT);
  Wire.onReceive(receiveEvent);


  I2Creservieren();
  I2Cpwm.begin();

  I2Cpwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
  //delay(100);
  byte tiefe = 57;
  I2Cpwm.setPWM(0, 0, map(tiefe, 0, 57, 170, 510));
  
  I2Cfreigeben();

}




void loop() {

  if ((millis() >= (letztesMal + Zeitbedarf)) && (fertig != 1)) {
    letztesMal = millis();
    Zeitbedarf = vregler();
    Serial.println("vtRegler ausgefuehrt");
    Serial.println(Zeitbedarf);

  }

  if (neuerBefehl == 1) {
    neuerBefehl = 0;

    Befehleinordnen();

  }

  if (millis() >= letztesMalSOC + 60) { // Akku könnte schnellsten bei 1,5Ah und 10A Leistungfluss in ca. 10 min gelert werden. Somit wäre der schnellste Verfall 1% in 6 sek. (zum Testen auf 60 gesetzt=> später wieder auf 6000)
    letztesMalSOC = millis();
    SOC = constrain(constrain(map(analogRead(7), 711, 856, 0, 100), SOC - 1, SOC + 1), 0, 100);
    //Serial.print("SOC:");
    //Serial.println(SOC);
  }


  //Serial.println("Dauer");
  //Serial.println(Zeitbedarf);
  /*  // //*********************************HardwareSerial3-ESP*********************************
    if (Serial3.available() > 0) {
      while (Serial3.available()) {

        long check;
        check = Serial3.parseInt();
        Serial3.readBytes(Befehle.by, 13);

        if (check == receivecheck() || check == -receivecheck()) {

          umlenken();

        } else {

        }
      }

    }
    //
  */

}

//************************************I2C-Bus***********************************************
void receiveEvent(int howMany) {
  int w = 0;
  while (Wire.available())
  {
    if (w <= 7) {
      Befehle.by[w] = Wire.read();
    }
    else {
      Wire.read();
    }
    w += 1;
  }
  neuerBefehl = 1;

}


void Befehleinordnen() {
  noInterrupts();

  //  Serial.println(Befehle.Fahrbefehl.fuehrungsbyte);
  //  Serial.println(Befehle.Fahrbefehl.vsollneu);
  //  Serial.println(Befehle.Fahrbefehl.ssollneu);
  //  Serial.println(Befehle.Fahrbefehl.phisollneu);
  //  Serial.println(Befehle.Fahrbefehl.priosollneu);
  //  Serial.println(Befehle.Fahrbefehl.typsollneu);
  fertig = 0;
  Position = 0;
  if ((Befehle.Fahrbefehl.priosollneu == 255) || (priosoll[0] == 255)) {
    Position = 0;

  }
  else
  {
    while ((Befehle.Fahrbefehl.priosollneu <= priosoll[Position]) && (Position <= 19)) {
      Position++;
    }

    // Serial.print("Position");
    // Serial.println(Position);
  }
  if (Position == 0) {
    Zeitbedarf = 0;
  }
  if (Position <= 19) {
    //Serial.println("Modulotest");
    //  Serial.println(Befehle.Fahrbefehl.typsollneu % 2);
    if ((Befehle.Fahrbefehl.typsollneu % 2) == 1) {
      for (int r = 19; r > Position; r--) {
        vsoll[r] = 0;
        ssoll[r] = 0;
        phisoll[r] = 0;
        priosoll[r] = 0;
        typsoll[r] = 0;
      }
      //  Serial.print("Erfolgreich an Position:"); Serial.print(Position);
    }
    else {
      for (int e = 19; e > Position; e--)
      {
        vsoll[e] = vsoll[e - 1];
        ssoll[e] = ssoll[e - 1];
        phisoll[e] = phisoll[e - 1];
        priosoll[e] = priosoll[e - 1];
        typsoll[e] = typsoll[e - 1];
      }
      //  Serial.print("Erfolgreich an Position:"); Serial.print(Position);
    }
    vsoll[Position] = Befehle.Fahrbefehl.vsollneu;
    ssoll[Position] = Befehle.Fahrbefehl.ssollneu;
    phisoll[Position] = Befehle.Fahrbefehl.phisollneu;
    priosoll[Position] = Befehle.Fahrbefehl.priosollneu;
    typsoll[Position] = Befehle.Fahrbefehl.typsollneu;
  }
  interrupts();

}




long vregler()
{




  vAussen = vsoll[0];
  if ((phisoll[0] == 0) && (vsoll[0] != 0))
  {
    vInnen = vsoll[0];
    dauer = ssoll[0] * 1000.0 / vsoll[0];
  }
  else if ((phisoll[0] != 0) && (vsoll[0] != 0)) {
    sRadius = abs(ssoll[0] * 360.0 / (phisoll[0] * 2 * 3.14));
    vInnen = vAussen * (sRadius - Spurweite / 2) / (sRadius + Spurweite / 2);
    sInnen = (sRadius - Spurweite / 2) * ((double) phisoll[0] / 360.0 * 2 * 3.14); //Kann glaube ich aus auskommentiert werden
    sAussen = (sRadius + Spurweite / 2) * ((double) phisoll[0] / 360.0 * 2 * 3.14);
    dauer = abs(sAussen * 1000.0 / vAussen);
  }
  if ((vsoll[0] > 0) && (ssoll[0] >= 0)) {
    if (phisoll[0] < 0) {
      vWerte.vSammlung.vRechts = vAussen;
      vWerte.vSammlung.vLinks = vInnen;
    }
    else if (phisoll[0] > 0) {
      vWerte.vSammlung.vRechts = vInnen;
      vWerte.vSammlung.vLinks = vAussen;
    }
    else
    {
      vWerte.vSammlung.vLinks = vsoll[0];
      vWerte.vSammlung.vRechts = vsoll[0];
    }
  }
  else if ((vsoll[0] < 0) && (ssoll[0] <= 0)) {  //Überide kleiner Gleich muss nochmal nachgedacht werden
    if (phisoll[0] < 0) {
      vWerte.vSammlung.vRechts = vInnen;
      vWerte.vSammlung.vLinks = vAussen;
    }
    else if (phisoll[0] > 0) {
      vWerte.vSammlung.vRechts = vAussen;
      vWerte.vSammlung.vLinks = vInnen;
    }
    else
    {
      vWerte.vSammlung.vLinks = vsoll[0];
      vWerte.vSammlung.vRechts = vsoll[0];
    }
  }
  else if ((vsoll[0] == 0) && (ssoll[0] == 0))
  { fertig = 1;
    dauer = 0;
    vWerte.vSammlung.vLinks = 0; //[in 1/10 mm]
    vWerte.vSammlung.vRechts = 0; //[in 1/10 mm]
  }
  else if ((vsoll[0] == 0) && (ssoll[0] < 0))
  { dauer = ssoll[0];
    vWerte.vSammlung.vLinks = 0; //[in 1/10 mm]
    vWerte.vSammlung.vRechts = 0; //[in 1/10 mm]
  }
  else
  {
    vWerte.vSammlung.vLinks = 0; //[in 1/10 mm]
    vWerte.vSammlung.vRechts = 0; //[in 1/10 mm]
  }

  if (I2Creservieren() == 1) {


    Wire.beginTransmission(20); // transmit to device #8
    Wire.write(vWerte.vByte[0]);
    Wire.write(vWerte.vByte[1]);
    Wire.write(vWerte.vByte[2]);
    Wire.write(vWerte.vByte[3]);// sends one byte
    Wire.endTransmission();    // stop transmitting

    Wire.beginTransmission(21); // transmit to device #8
    Wire.write(vWerte.vByte[0]);
    Wire.write(vWerte.vByte[1]);
    Wire.write(vWerte.vByte[2]);
    Wire.write(vWerte.vByte[3]);// sends one byte
    Wire.endTransmission();    // stop transmitting
    I2Cfreigeben();


  }; //20 rechts 21 links

  for (int t = 0; t < 19; t++) {
    vsoll[t] = vsoll[t + 1];
    ssoll[t] = ssoll[t + 1];
    phisoll[t] = phisoll[t + 1];
    priosoll[t] = priosoll[t + 1];
    typsoll[t] = typsoll[t + 1];
  }
  vsoll[19] = 0;
  ssoll[19] = 0;
  phisoll[19] = 0;
  priosoll[19] = 0;
  typsoll[19] = 0;


  return dauer;

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
