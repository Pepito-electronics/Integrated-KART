
#include <ACAN2515.h>
#include <HCSR04.h>

/*
 * Broches pour le chip select et l'interruption du MCP2515
 */

static const byte MCP2515_SCK  = 26 ; // SCK input of MCP2517 
static const byte MCP2515_MOSI = 19 ; // SDI input of MCP2517  
static const byte MCP2515_MISO = 18 ; // SDO output of MCP2517 

static const byte MCP2515_CS  = 17; // CS input of MCP2515 (adapt to your design) 
static const byte MCP2515_INT =  21 ; // INT output of MCP2515 (adapt to your design)

/*
 * L'objet pour piloter le MCP2515. SPI designe l'objet
 * utilise pour la connexion SPI car sur certaines cartes
 * notamment les Teensy, il peut y avoir plusieurs SPI.
 */
ACAN2515 controleurCAN(MCP2515_CS, SPI, MCP2515_INT);

/*
 * La frequence du quartz du MCP2515 en hertz.
 */
static const uint32_t FREQUENCE_DU_QUARTZ = 20UL * 1000UL * 1000UL ; // 20 MHz

/*
 * La fréquence du bus CAN
 */
static const uint32_t FREQUENCE_DU_BUS_CAN = 125ul * 1000ul;

/*
 * Un objet pour le message CAN. Par defaut c'est un message
 * standard avec l'identifiant 0 et aucun octet de donnees
 */
CANMessage messageCANReception;
CANMessage messageCANEmission1;

// PIN des différente Capteurs/Actionneurs

const int LDR = 15;   //LDR ou photorésistance
const int hallPin = 22; //Capteur effet hall
const int pharearriere = 23;  //phare arriere 
const int clignoteurG = 12; //clignoteur gauche
const int clignoteurD = 14;  //clignoteur droite
UltraSonicDistanceSensor distanceSensor(4, 5);   //PIN TRIGGER et ECHO du Ultrason

//Différentes variables
bool parking;
int lumi;
int sensorValue;
bool flag_sensor;
uint8_t stateLumi = 0;
bool oldstate;
int distance;

//Fonction pour allumer les clignoteurs si on freine (que le capteur ne detecte plus l'aimant)
void Halldetect()
{
  if (sensorValue == LOW)
  {
    Serial.print("Halldetect");
    digitalWrite(clignoteurG,sensorValue);
    digitalWrite(clignoteurD,sensorValue); 
  }
  else if (sensorValue == HIGH)
  {
    digitalWrite(clignoteurG,sensorValue);
    digitalWrite(clignoteurD,sensorValue);
  }
}

void setup()
{
  //Initialise le Serial
  Serial.begin(115200);
  //Initialise le SPI
  SPI.begin (MCP2515_SCK, MCP2515_MISO, MCP2515_MOSI) ;
  Serial.println("Configuration du MCP2515");
  //Fixe la vitesse du bus a 125 kbits/s 
  ACAN2515Settings reglages(FREQUENCE_DU_QUARTZ, FREQUENCE_DU_BUS_CAN);
  // Demarre le CAN 
  const uint16_t codeErreur = controleurCAN.begin(reglages, [] { controleurCAN.isr(); } );
  // Verifie que tout est ok 
  if (codeErreur == 0) {
    Serial.println("Recepteur: configuration ok");
  }
  else {
    Serial.println("Recepteur: Probleme de connexion");
    while (1); 
  }

  
  //Initialise les broche des LEDs
  pinMode(pharearriere, OUTPUT);
  digitalWrite(pharearriere,LOW);
  pinMode(clignoteurD, OUTPUT);
  digitalWrite(clignoteurD,LOW);
  pinMode(clignoteurG, OUTPUT);
  digitalWrite(clignoteurG,LOW);
  //Configure la longueur de la trame d'envoi
  messageCANEmission1.len = 3;
  //met le flag à zero
  stateLumi = 0;
}
//fonction loop
void loop()
{
  //Reception de donnee du CAN 
  if (controleurCAN.receive(messageCANReception)) {
    /* Un message CAN est arrive */
    static uint32_t numero = 0;
    controleurCAN.receive(messageCANReception) ;
    Serial.println(messageCANReception.data[1]);
    //on reçois l'etat de parking
    parking = messageCANReception.data[1];
  }
  //lecture de l'etat du capteur effet hall
  sensorValue = digitalRead(hallPin);
  //fonction du capteur effet hall 
  Halldetect();
  //SI parking passe à 1, on commence à envoyer des données de notre ultrason
  if (parking == 1)
  {
    //envoi de la donnee de la distance entre nous et un obstacle via le capteur ultrasons
    messageCANEmission1.data[1] = distance;
    //vérification que le message a bien été envoyé
    const bool ok = controleurCAN.tryToSend(messageCANEmission1);
        if (ok) 
        {
          Serial.print("message ");
          Serial.print("Distance ");
          Serial.println(" envoyee !");
        }
  }
  //lecture de la valeur de la photoresistance
  lumi = analogRead(LDR);
  Serial.println(lumi);
  /* Dans les lignes ci-dessous, on envoie notre message 
  seulement lorsqu'il y a un changement d'etat de notre 
  LDR, c'est à dire si la valeur de la luminosité passe 
  au dessus ou en dessous de 2000. Cela allege le 
  microcontrôleur qui ne dois donc pas envoyer des messages 
  en permanence    */
  if (lumi < 2000)
  {
    stateLumi = 1;
  }
  else if (lumi >= 2000)
  {
    stateLumi = 0;
  }
  //si la luminosité est en dessous de 2000 et qu'elle était avant au dessus de 2000
  if (stateLumi == 1 && oldstate == 0)
  {
    messageCANEmission1.data[0] = stateLumi;
    const bool ok = controleurCAN.tryToSend(messageCANEmission1);
      if (ok) 
      {
        Serial.print("message ");
        Serial.print("Allumer ");
        Serial.println(" envoye !");
      }
    //on allume en meme temps les phare arriere
    digitalWrite(pharearriere,HIGH);
  }
  //si la luminosité est au dessus de 2000 et qu'elle était avant en dessous de 2000
  else if (stateLumi == 0 && oldstate == 1)
  {
    messageCANEmission1.data[0] = stateLumi;
    const bool ok = controleurCAN.tryToSend(messageCANEmission1);
      if (ok) 
      {
        Serial.print("message ");
        Serial.print("Eteindre ");
        Serial.println(" envoye !");
      }
    //on eteind en meme temps les phare arriere
    digitalWrite(pharearriere,LOW);
  }
  //on sauvegarde la le flag precedent 
  oldstate = stateLumi;
  delay(1000);
}