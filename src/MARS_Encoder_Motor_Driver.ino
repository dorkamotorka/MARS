#include <digitalWriteFast.h>
#include "Arduino.h"
#include <PID_v1.h>
#include <Wire.h>

/*
  MARS Encoder Motor Driver

  Basiert auf der Hardware des ME Encoder Motor drivers

  Liest über den I2C Bus Geschwindigkeiten der Räder und steuert
  entweder beide Linken oder beide Rechten Räder entsprechen an.

  Es wird nur die Geschwindigkeit geregelt, keine Verarbeitung von
  Fahrbefehlen
  Optional, gibt die absolut gefahrene Strecke zurück



  PWM Pin wird mit PWM gepulst
*/
#define Rechts 0 // Für Linke Seite hier eine 0 einsetzen

//Entgegen der Angabe auf dem Schaltplan des ME PIN 14 und 15 getauscht dies
//führt zu gleicher Drehrichtung auf beiden Kanälen
//Motor1
#define AIN1 14 //Analog Pin A0, getauscht
#define AIN2 15 //Analog Pin A1, getauscht
#define APWM 6
//Motor2
#define BIN1 16 //Analog Pin A2, 
#define BIN2 17 //Analog Pin A3, 
#define BPWM 5

//Encoder Pins
// Quadrature encoders
#define c_1_A 3 //Motor 1 Spur A TachoX1 = Net1 = PD3 
#define c_1_B 10 //Motor 1 Spur B TachoX2 = Net2 = PB2  
volatile bool BSet1;
volatile long Ticks1 = 0;

#define c_2_A 2 //Motor 1 Spur A TachoX3 = Net3 = PD2
#define c_2_B 12 //Motor 1 Spur B TachoX4 = Net4 = PB4 
volatile bool BSet2;
volatile long Ticks2 = 0;

//Für die Regelung

long Ticksneu1 = 0;
long Ticksneu2 = 0;
long Ticksalt1 = 0;
long Ticksalt2 = 0;

long v1 = 0; //Geschwindigkeit Motor 1 in 1/10mm/s
long v2 = 0; //Geschwindigkeit Motor 2 in 1/10mm/s

//für den PID Regler
double vSollReglerEigenes = 0;
double vSollReglerAnderes = 0;
double vSollEigenes = 0;
double vSollAnderes = 0;
double vIst1, PWMWert1;
double vIst2, PWMWert2;
double Kp = 0.10, Ki = 1.5, Kd = 0;

int DeltatRegelung = 20; //Zeit in ms zwischen den Regelungen
int Radumfang = 314; //in mm

int Beschleunigung = 5000; //(1/10)mm/s^2
int DeltavAnstieg = (Beschleunigung / 1000) * DeltatRegelung;   //Anstieg in (1/10mm)/s pro Regelungsdurchlauf

PID PID1(&vIst1, &PWMWert1, &vSollReglerEigenes, Kp, Ki, Kd, DIRECT);
PID PID2(&vIst2, &PWMWert2, &vSollReglerEigenes, Kp, Ki, Kd, DIRECT);
//Zeitvariablen für Serielle Testversendung
long letztesmal;
long diesesmal;

int dauer;
//Komtainer für den I2c Empfang
union I2CkontainerEmpfang {
  byte vByte[4];
  struct {
    int vRechts;
    int vLinks;
  } vSammlung;
} vWerte;

//Komtainer für die Serielle Versendung
union I2CkontainerSenden {
  byte b[6];
  struct {
    int vIstMittel[1];
    long sIstMittel[1];
  } SendenSammlung;
} SendWerte;

void setup() {
  // put your setup code here, to run once:
  if (Rechts == 1) {
    Wire.begin(20);
  }
  else
  {
    Wire.begin(21);
  }
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent); // register event
  Serial.begin(115200);
  // Quadrature encoders
  // LV 0
  pinMode(c_1_A, INPUT_PULLUP);      // sets pin A as input
  pinMode(c_1_B, INPUT_PULLUP);      // sets pin A as input
  attachInterrupt(digitalPinToInterrupt(c_1_A), Handle1_Interrupt, RISING);

  // LH 1
  pinMode(c_2_A, INPUT_PULLUP);      // sets pin A as input
  pinMode(c_2_B, INPUT_PULLUP);      // sets pin A as input
  attachInterrupt(digitalPinToInterrupt(c_2_A), Handle2_Interrupt, RISING);
  //Aktiviert den PID Regler
  PID1.SetOutputLimits(-255, 255);
  PID2.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(DeltatRegelung);
  PID2.SetSampleTime(DeltatRegelung);
  PID1.SetMode(AUTOMATIC);
  PID2.SetMode(AUTOMATIC);
  vWerte.vSammlung.vRechts = 0;
  vWerte.vSammlung.vLinks = 0;


}

void loop() {
  // put your main code here, to run repeatedly:




  diesesmal = millis();
  if ((diesesmal - letztesmal) >= DeltatRegelung) {
    letztesmal = diesesmal;
    dauer = millis();
    Ticksneu1 = Ticks1 - Ticksalt1;
    Ticksneu2 = Ticks2 - Ticksalt2;
    Ticksalt1 = Ticks1;
    Ticksalt2 = Ticks2;
    //Umfang Rad ca. 314mm
    v1 = Ticksneu1 * Radumfang / 6;
    v2 = Ticksneu2 * Radumfang / 6;

    //Eigentlich bei Frequenz der Regelung von 50Hz (20ms) v1=Ticksneu1*50/100/30*3140
    //                                                     v1=Ticks pro Zyklus*Frequenz/Ticks pro Encoderumdrehung/Übersetzung*Weg pro Umderehung(in 10/mm)
    vIst1 = v1; // Casten von Int zu Double
    vIst2 = v2; // Casten von Int zu Double

    //Wahl der Seite
    if (Rechts == 1) {
      noInterrupts();
      vSollEigenes = vWerte.vSammlung.vRechts * 10;
      vSollAnderes = vWerte.vSammlung.vLinks * 10;
      interrupts();
    }
    else
    {
      noInterrupts();
      vSollEigenes = vWerte.vSammlung.vLinks * 10;
      vSollAnderes = vWerte.vSammlung.vRechts * 10;
      interrupts();
    }
    //Rauf und Runterrampen der Räder (s. dazugehöriges Excel Sheet (auf Anfrage erhältlich))
    if (vSollAnderes == vSollReglerAnderes) {
      vSollReglerEigenes = constrain(vSollEigenes, vSollReglerEigenes - DeltavAnstieg, vSollReglerEigenes + DeltavAnstieg);
    }
    else
    {

      vSollReglerEigenes = constrain(vSollEigenes, vSollReglerEigenes - (DeltavAnstieg * constrain(abs(vSollReglerEigenes - vSollEigenes) / abs(vSollReglerAnderes - vSollAnderes), 0, 1)), vSollReglerEigenes + (DeltavAnstieg * constrain(abs(vSollReglerEigenes - vSollEigenes) / abs(vSollReglerAnderes - vSollAnderes), 0, 1)));

      vSollReglerAnderes = constrain(vSollAnderes, vSollReglerAnderes - (DeltavAnstieg * constrain(abs(vSollReglerAnderes - vSollAnderes) / abs(vSollReglerEigenes - vSollEigenes), 0, 1)), vSollReglerAnderes + (DeltavAnstieg * constrain(abs(vSollReglerAnderes - vSollAnderes) / abs(vSollReglerEigenes - vSollEigenes), 0, 1)));
    }

    //PID Regler berechnen lassen
    PID1.Compute();
    PID2.Compute();




    //Deaktivieren der Motoren im Stillstand Akku, Treiber, Motoren und Ohren (hohes Fiepen) zu schonen.
    if (abs(v1) < 50 && vSollReglerEigenes == 0) {
      PWMWert1 = 0;
      PID1.SetMode(MANUAL);
    }
    else {
      PID1.SetMode(AUTOMATIC);
    }


    if (abs(v2) < 50 && vSollReglerEigenes == 0) {
      PWMWert2 = 0;
      PID2.SetMode(MANUAL);
    }
    else {
      PID2.SetMode(AUTOMATIC);
    }

    //Ansteuern der Richtungspins (PWM Wert größer 0 gleich vorwärts; kleiner 0 gleich Rückwärt)
    if (PWMWert1 > 0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    }
    else
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }

    if (PWMWert2 > 0) {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    }
    else
    {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    }
    // Schreiben der PWM Signale

    analogWrite(APWM, abs(PWMWert1));
    analogWrite(BPWM, abs(PWMWert2));

    //Speichern der Fahrstrecke und Geschwindigkeit zur Ausgabe über einen Bus
      noInterrupts();
    SendWerte.SendenSammlung.vIstMittel[0] = (v1 + v2) / 20; //Eigentlich durch 2 und durch 10 (Es wird in mm/s gesendet; v1 und v2 sind aber in 1/10mm/s)
    SendWerte.SendenSammlung.sIstMittel[0] = (Ticks1 + Ticks2) * Radumfang / 30 / 100 / 2;
      interrupts();
    //SendWerte.SendenSammlung.vIstMittel[0] = 100;
    //SendWerte.SendenSammlung.sIstMittel[0] = 10000;

    //Serial.print(SendWerte.SendenSammlung.vIstMittel[0]);
    //Serial.println(SendWerte.SendenSammlung.sIstMittel[0]);

    //Serial.print("v I2C mal 10 Rechts + Links#");
    //Serial.print(SendWerte.SendenSammlung.vIstMittel);
    //  Serial.print("#");
    //Serial.println(SendWerte.SendenSammlung.sIstMittel);

    /*Für das Debugging in der Konsole
      Serial.println("v I2C mal 10 Rechts + Links");
      Serial.println(vWerte.vSammlung.vRechts[0] * 10);
      Serial.println(vWerte.vSammlung.vLinks[0] * 10);
      Serial.println("v Soll Eig + Anderes");
      Serial.println(vSollEigenes);
      Serial.println(vSollAnderes);
      Serial.println("v Regler Eig + Anderes");
      Serial.println(vSollReglerEigenes);
      Serial.println(vSollReglerAnderes);
      Serial.println("PWM Werte");
      Serial.println(PWMWert1);
      Serial.println(PWMWert2);
      Serial.println("v_Ist");
      Serial.println(v1);
      Serial.println(v2);
      Serial.println("DeltavAnstieg");
      Serial.println(DeltavAnstieg);
      Serial.println("Ticks");
      Serial.println(Ticks1);
      Serial.println(Ticks2);
      Serial.println("dauer");
      Serial.println(millis() - dauer);
    */

    /*
        //Für das Debugging über KOnsole und z.B. Excel
        Serial.print(vWerte.vSammlung.vRechts[0] * 10);
        Serial.print("#");
        Serial.print(vWerte.vSammlung.vLinks[0] * 10);
        Serial.print("#");
        Serial.print(vSollEigenes);
        Serial.print("#");
        Serial.print(vSollAnderes);
        Serial.print("#");
        Serial.print(vSollReglerEigenes);
        Serial.print("#");
        Serial.print(vSollReglerAnderes);
        Serial.print("#");
        Serial.print(PWMWert1);
        Serial.print("#");
        Serial.print(PWMWert2);
        Serial.print("#");
        Serial.print(v1);
        Serial.print("#");
        Serial.print(v2);
        Serial.print("#");
        Serial.print(DeltavAnstieg);
        Serial.print("#");
        Serial.print(Ticks1);
        Serial.print("#");
        Serial.print(Ticks2);
        Serial.print("#");
        Serial.println(millis() - dauer);

    */

  }
}
//Encoderauswertung Motor 1
void Handle1_Interrupt()
{
  noInterrupts();
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  BSet1 = digitalReadFast(c_1_B);   // read the input pin

  // and adjust counter + if A leads B
#ifdef LeftEncoderIsReversed
  Ticks1 += BSet1 ? -1 : +1;
#else
  Ticks1 -= BSet1 ? -1 : +1;
#endif
  interrupts();
}
//Encoderauswertung Motor 2
void Handle2_Interrupt()
{
  noInterrupts();
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  BSet2 = digitalReadFast(c_2_B);   // read the input pin

  // and adjust counter + if A leads B
#ifdef RightEncoderIsReversed
  Ticks2 += BSet2 ? -1 : +1;
#else
  Ticks2 -= BSet2 ? -1 : +1;
#endif
  interrupts();
}
//Lesen der I2C Geschwindigkeiten
void receiveEvent(int howMany) {
  int i = 0;
  while (0 < Wire.available()) { // loop through all but the last
    if (i <= 3) {
      vWerte.vByte[i] = Wire.read();
      i += 1;
    }
    else {
      Wire.read();
    }
  }
  vWerte.vSammlung.vRechts = constrain( vWerte.vSammlung.vRechts, -500, 500);
  vWerte.vSammlung.vLinks = constrain( vWerte.vSammlung.vLinks, -500, 500);
}

void requestEvent() {
  Wire.write(SendWerte.b, 6);

  // as expected by master
}

