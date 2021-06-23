#include <Wire.h>
#include <ros.h>
#include <Adafruit_PWMServoDriver.h>
#include <imega_arduino/custom.h>

// Speicher für die Befehle (Beschreibung s. union Befehle)(Der erste Befehl (Arrayplatz=0) wird zuerst verarbeitet)
int vsoll[20] =     {10,   0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Speed of the faster wheel [in 1 mm per second]
int ssoll[20] =     {10,   0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Driving distance of the robot midpoint for the command [in 1 mm]
int phisoll[20] =   {0,    0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Clockwise turing angle for the command seen from above in degrees
byte priosoll[20] = {1,    0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Priority of the command (255 = execudet immediately)(1 lowest prioritiy)
byte typsoll[20] =  {0,    0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 1 to Delete all commandy with lower priority;

// Container for sending  für den I2c Versandt
union I2Ckontainer {
  byte vByte[4];
  struct {
    int vRechts;
    int vLinks;
  } vSammlung;
} vWerte;


double Spurweite = 400; 	// Eigentlich 284 aber der Wert hat durch Schlupf eher den Solldrehwinkel wergeben [in 1/10 mm]


int Position = 0; 		// Position im Fahrbefehl Array an dem ein neu angekommener Befehl in Abhängigkeit seiner Priorität eingeordnet wird.
long Zeitbedarf = 0; 		// Berechnete Zeit die ein Fahrbefehl dauert. Anschließend kommt der nächste.
long dauer = 0;   		// Variable für den Zeitbedart in der vregler Funktion (kann ggf. auch dorthin verschoben werden)

long letztesMal = 0; 		// Zeit in ms wann der letzte Fahrbefehl aufgeführt wurde


double vInnen = 0; 		// Geschwindigkeit des Kurveninneren Rades
double vAussen = 0; 		// Geschwindigkeit des Kurvenäußeren Rades
double sRadius = 0; 		// Radius der Kurvenfahrt (Fahrzeugmittelpunkt)
double sInnen = 0;
double sAussen = 0;

int fertig = 0;



//Beginn of needed functions for driving

void driveAngle(int vsollNeu, int ssollNeu, int phisollNeu, byte priosollNeu, byte typsollNeu) {
  noInterrupts();
  fertig = 0;
  Position = 0;

  if (priosollNeu == 255 ) {
    Position = 0;
  } else {
    while ((priosollNeu <= priosoll[Position]) && (Position <= 19)) {
      Position++;
    }

  }

  if (Position == 0) {
    Zeitbedarf = 0;
  }

  if (Position <= 19) {
    if ((typsollNeu % 2) == 1) {
      for (int r = 19; r > Position; r--) {
        vsoll[r] = 0;
        ssoll[r] = 0;
        phisoll[r] = 0;
        priosoll[r] = 0;
        typsoll[r] = 0;
      }
    } else {
      for (int e = 19; e > Position; e--)
      {
        vsoll[e] = vsoll[e - 1];
        ssoll[e] = ssoll[e - 1];
        phisoll[e] = phisoll[e - 1];
        priosoll[e] = priosoll[e - 1];
        typsoll[e] = typsoll[e - 1];
      }

    }
    vsoll[Position] = vsollNeu;
    ssoll[Position] = ssollNeu;
    phisoll[Position] = phisollNeu;
    priosoll[Position] = priosollNeu;
    typsoll[Position] = typsollNeu;
  }
  interrupts();
}

//Beginn of needed functions for driving
void driveDirect(int speedLeft, int speedRight) {
  //Container for sending Data to the I2C Bus.
  union I2Ckontainer {
    byte vByte[4];
    struct {
      int vRechts;
      int vLinks;
    } vSammlung;
  } vWerte;
  vWerte.vSammlung.vRechts = speedRight;
  vWerte.vSammlung.vLinks = speedLeft;
  Wire.beginTransmission(0x11); // transmit to device #8
  Wire.write(vWerte.vByte[0]);
  Wire.write(vWerte.vByte[1]);
  Wire.write(vWerte.vByte[2]);
  Wire.write(vWerte.vByte[3]);// sends one byte
  Wire.endTransmission();    // stop transmitting
  Wire.beginTransmission(0x12); // transmit to device #8
  Wire.write(vWerte.vByte[0]);
  Wire.write(vWerte.vByte[1]);
  Wire.write(vWerte.vByte[2]);
  Wire.write(vWerte.vByte[3]);// sends one byte
  Wire.endTransmission();    // stop transmitting
}


long vregler() {
  vAussen = vsoll[0];

  if ((phisoll[0] == 0) && (vsoll[0] != 0)) {
    vInnen = vsoll[0];
    dauer = ssoll[0] * 1000.0 / vsoll[0];
  } else if ((phisoll[0] != 0) && (vsoll[0] != 0)) {
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
    } else if (phisoll[0] > 0) {
      vWerte.vSammlung.vRechts = vInnen;
      vWerte.vSammlung.vLinks = vAussen;
    } else {
      vWerte.vSammlung.vLinks = vsoll[0];
      vWerte.vSammlung.vRechts = vsoll[0];
    }
  }
  else if ((vsoll[0] < 0) && (ssoll[0] <= 0)) {  //Überide kleiner Gleich muss nochmal nachgedacht werden
    if (phisoll[0] < 0) {
      vWerte.vSammlung.vRechts = vInnen;
      vWerte.vSammlung.vLinks = vAussen;
    } else if (phisoll[0] > 0) {
      vWerte.vSammlung.vRechts = vAussen;
      vWerte.vSammlung.vLinks = vInnen;
    } else {
      vWerte.vSammlung.vLinks = vsoll[0];
      vWerte.vSammlung.vRechts = vsoll[0];
    }
  }
  else if ((vsoll[0] == 0) && (ssoll[0] == 0)) {
    fertig = 1;
    dauer = 0;
    vWerte.vSammlung.vLinks = 0; //[in 1/10 mm]
    vWerte.vSammlung.vRechts = 0; //[in 1/10 mm]
  } else if ((vsoll[0] == 0) && (ssoll[0] < 0)) {
    dauer = ssoll[0];
    vWerte.vSammlung.vLinks = 0; //[in 1/10 mm]
    vWerte.vSammlung.vRechts = 0; //[in 1/10 mm]
  } else {
    vWerte.vSammlung.vLinks = 0; //[in 1/10 mm]
    vWerte.vSammlung.vRechts = 0; //[in 1/10 mm]
  }

  if (1) {
    Wire.beginTransmission(0x11); // transmit to device #8
    Wire.write(vWerte.vByte[0]);
    Wire.write(vWerte.vByte[1]);
    Wire.write(vWerte.vByte[2]);
    Wire.write(vWerte.vByte[3]);// sends one byte
    Wire.endTransmission();    // stop transmitting

    Wire.beginTransmission(0x12); // transmit to device #8
    Wire.write(vWerte.vByte[0]);
    Wire.write(vWerte.vByte[1]);
    Wire.write(vWerte.vByte[2]);
    Wire.write(vWerte.vByte[3]);// sends one byte
    Wire.endTransmission();    // stop transmitting
  }

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

/* ROS Callback */
void commanderCB(const imega_arduino::custom & msg) {
	// Wait 10s such that everything is connected
	int speed = msg.data[0]; //[mm/s] ~ max 0.5m/s
	int dist = msg.data[1];
	int angle = msg.data[2] ; // degress
	// priority - priority of the command
	// delete - delete all commands with lower priority
    	driveAngle(speed, dist, angle, 5, 1); // speed, distance[mm], angle(clockwise from above), priority, delete
}

/* ROS variables */
ros::NodeHandle nh;
ros::Subscriber<imega_arduino::custom> controller("/vel_cmd", &commanderCB);

/* Setup function */
void setup()
{
  // Beginn of needed functions in the setup().
  Wire.begin(); // Hauptcontroller hat die Adresse 8 dies ist auch das Führugnsbyte
  nh.initNode();
  nh.subscribe(controller);
}

/* Main loop */
void loop() {

  // DO NOT REMOVE
  // Beginn of needed functions in the loop(). This part of code need to be executed as often as possible. A frequency of 100Hz is great.
  if ((millis() >= (letztesMal + Zeitbedarf)) && (fertig != 1)) {
   letztesMal = millis();
   Zeitbedarf = vregler(); // This function manages/updates command buffer
  }
  // DO NOT REMOVE

  nh.spinOnce();
}
