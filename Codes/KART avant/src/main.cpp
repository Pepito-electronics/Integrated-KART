// CODE CARTE AV ==> HMI + CPT

#include <HCSR04.h>
#include <ACAN2515.h>
#include <Arduino.h>
#include <Nextion.h> 
#include <SPI.h> 

#define Front_light 4
#define HALL_PIN 23
#define Klaxon 16
#define ultraS_pin 25
#define ANALOG_PIN 34

UltraSonicDistanceSensor distanceSensor(32, 33);  // Initialize sensor that uses digital pins 13 and 12.


static const byte MCP2515_SCK  = 26 ; // SCK input of MCP2517 
static const byte MCP2515_MOSI = 19 ; // SDI input of MCP2517  
static const byte MCP2515_MISO = 18 ; // SDO output of MCP2517 

static const byte MCP2515_CS  = 17 ; // CS input of MCP2515 (adapt to your design) 
static const byte MCP2515_INT =  21 ; // INT output of MCP2515 (adapt to your design)

ACAN2515 controleurCAN(MCP2515_CS, SPI, MCP2515_INT);
static const uint32_t FREQUENCE_DU_QUARTZ = 20ul * 1000ul * 1000ul;
static const uint32_t FREQUENCE_DU_BUS_CAN = 125ul * 1000ul;

CANMessage messageCANEmission;
CANMessage messageCANReception1;


// Declare objects that we are going to read from the display. This includes buttons, sliders, text boxes, etc:
// Format: <type of object> <object name> = <type of object>(<page id>, <object id>, "<object name>");
NexButton b0 = NexButton(4, 2, "b0");  // Button added
NexButton b2 = NexButton(0, 4, "b2");  // Button added
NexButton b1 = NexButton(0, 3, "b1");  // Button added
NexButton b25 = NexButton(3, 17, "b25");  // Button added
NexButton b10 = NexButton(3, 15, "b10");  // Button added
NexButton b16 = NexButton(3, 4, "b16");  // Button added
NexButton b12 = NexButton(1, 1, "b12");  // Button added
NexButton b11 = NexButton(2, 3, "b11");  // Button added
NexButton b21 = NexButton(4, 5, "b21");  // Button added
NexButton b30 = NexButton(4, 4, "b30");  // Button added
NexButton b33 = NexButton(4, 6, "b33");  // Button added
NexButton b34 = NexButton(4, 7, "b34");  // Button added
NexButton b35 = NexButton(4, 8, "b35");  // Button added


char buffer[100] = {0};  
int CurrentPage = 0;  // Create a variable to store which page is currently loaded

NexTouch *nex_listen_list[] = 
{
  &b0,  // Button added
  &b1,  // Button added
  &b2,  // Button added
  &b25,  // Button added
  &b10,  // Button added
  &b16,  // Button added
  &b12,  // Button added
  &b11,  // Button added
  &b21,  // Button added
  &b30,  // Button added
  &b33,  // Button added
  &b34,  // Button added
  &b35,  // Button added

  NULL  // String terminated
};  // End of touch event list

long tm1,tm2,tm3,tm_v;
int vit;
bool etat=false;
bool mvt;

//int index;
char command_line[32];
bool isCommandReady;

int vitesse,max_speed;
char serialbuffer[32];
char serialbuffer2[32];
char recptbuffer[8];
int cst,cst1,cst2;
int phareState, distance2;

int distance;

int sensorValue; 
bool flag_button,autoLight;

void calcul_v(){
  // lecture du capteur a Effet Hall
  sensorValue = digitalRead( HALL_PIN );
  // senseurValue = HIGH sans aimant
  // senseurValue = LOW  quand POLE SUD aimant
  sensorValue = not( sensorValue );
  if (sensorValue == HIGH)
  {
    flag_button = 1;
  }
  if (flag_button == 1 && sensorValue == LOW)
  {
    flag_button = 0;
    tm2 = millis();
    tm_v = tm2 - tm3;
    vit =(0.0007536 / tm_v)*3600000;   // avec 12cm de rayon
    tm3 = tm2;
  }
}

void b0PushCallback(void *ptr)  // Press event for button b0
{
  digitalWrite(Front_light,HIGH);
  //Serial.print("klaxon");
  //Serial.print("/n");
}  // End of press event
void b1PushCallback(void *ptr)  // Press event for button b0
{
  CurrentPage = 2;
  messageCANEmission.data[1]=1;
}  // End of press event
void b0PopCallback(void *ptr)  // Press event for button b0
{
  digitalWrite(Front_light,LOW);
  //Serial.print("klaxon");
  //Serial.print("/n");
}  // End of press event
void b2PushCallback(void *ptr)  // Press event for button b2
{
  CurrentPage = 3;  // Set variable as 3 so from now on arduino knows page 3 is loaded on the display
}  // End of press event
void b21PushCallback(void *ptr)  // Press event for button b2
{
  CurrentPage = 0;  // Set variable as 3 so from now on arduino knows page 3 is loaded on the display
}  // End of press event
void b30PushCallback(void *ptr)  // Press event for button b2
{
  CurrentPage = 3;  // Set variable as 3 so from now on arduino knows page 3 is loaded on the display
}  // End of press event
void b25PushCallback(void *ptr)  // Press event for button b2
{
  CurrentPage = 4;  // Set variable as 3 so from now on arduino knows page 3 is loaded on the display
}  // End of press event
void b10PushCallback(void *ptr)  // Press event for button b2
{
  CurrentPage = 0;  // Set variable as 3 so from now on arduino knows page 3 is loaded on the display
}  // End of press event
void b16PushCallback(void *ptr)  // Press event for button b2
{
  CurrentPage = 1;  // Set variable as 3 so from now on arduino knows page 3 is loaded on the display
}  // End of press event
void b12PushCallback(void *ptr)  // Press event for button b2
{
  CurrentPage = 0;  // Set variable as 3 so from now on arduino knows page 3 is loaded on the display
}  // End of press event
void b11PushCallback(void *ptr)  // Press event for button b2
{
  messageCANEmission.data[1]=0;
  CurrentPage = 0;  // Set variable as 3 so from now on arduino knows page 3 is loaded on the display
}  // End of press event
void b33PushCallback(void *ptr)  // Release event for dual state button bt0
{
  digitalWrite(Front_light,HIGH);

  //Serial.print("Front LIGHT");
  //Serial.print("/n");
}  // End of release event
void b34PushCallback(void *ptr)  // Release event for dual state button bt0
{
  digitalWrite(Front_light,LOW);

  //Serial.print("Back LIGHT");
  //Serial.print("/n");
}  // End of relea
void b35PushCallback(void *ptr)  // Release event for dual state button bt0
{
  autoLight = 1;
  //Serial.print("Back LIGHT");
  //Serial.print("/n");
}  // End of relea

////////////////////////// End of touch events

void hmiprint(char cmd[32]){
  Serial.print((String)cmd);  
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

void setup() {
    // put your setup code here, to run once:
  tm1=millis();
  max_speed=0;
  pinMode(Front_light,OUTPUT);
  pinMode(Klaxon,OUTPUT);
  Serial.begin(9600);

  /* Demarre le SPI */
  SPI.begin (MCP2515_SCK, MCP2515_MISO, MCP2515_MOSI) ;
  /* Configure le MCP2515 */
  Serial.println("Configuration du MCP2515");
  /* Fixe la vitesse du bus a 125 kbits/s */
  ACAN2515Settings reglages(FREQUENCE_DU_QUARTZ, FREQUENCE_DU_BUS_CAN);
  /* Demarre le CAN */
  const uint16_t codeErreur = controleurCAN.begin(reglages, [] { controleurCAN.isr(); } );
  /* Verifie que tout est ok */
  if (codeErreur == 0) {
    Serial.println("Configuration ok");
  }
  else {
    Serial.println("Probleme de connexion");
    //while (1); 
  }

  messageCANEmission.len = 2; 
  //messageCANReception1.len = 1; 

  // Register the event callback functions of each touch event:
  // You need to register press events and release events seperatly.
  // Format for press events: <object name>.attachPush(<object name>PushCallback);
  // Format for release events: <object name>.attachPop(<object name>PopCallback);
  b0.attachPush(b0PushCallback);  // Button press
  b0.attachPop(b0PopCallback);  // Button press
  b1.attachPush(b1PushCallback);  // Button press
  b2.attachPush(b2PushCallback);  // Button press
  b12.attachPush(b12PushCallback);
  b11.attachPush(b11PushCallback);
  b25.attachPush(b25PushCallback);
  b10.attachPush(b10PushCallback);
  b16.attachPush(b16PushCallback);
  b21.attachPush(b21PushCallback);
  b30.attachPush(b30PushCallback);
  b33.attachPush(b33PushCallback);  // Dual state button bt0 release
  b34.attachPush(b34PushCallback);  // Dual state button bt0 release
  b35.attachPush(b35PushCallback);  // Dual state button bt0 release

  // End of registering the event callback functions
}

void loop() {
  nexLoop(nex_listen_list);  // Check for any touch event
  if(CurrentPage == 0){  // If the display is on page 0, do the following:

  }
  if(CurrentPage == 1){  // If the display is on page 1, do the following:

  }
  if(CurrentPage == 2){  // If the display is on page 2, do the following:
    distance = 100-distanceSensor.measureDistanceCm();
    memset(serialbuffer,0,sizeof(serialbuffer));
    sprintf(serialbuffer,"j0.val=%d",distance);
    hmiprint(serialbuffer);
    memset(serialbuffer,0,sizeof(serialbuffer));
    sprintf(serialbuffer,"j1.val=%d",distance2);
    hmiprint(serialbuffer);
    delay(500);
  }
  if(CurrentPage == 3){  // If the display is on page 2, do the following:
    if(tm1<millis()){

      vitesse = analogRead(ANALOG_PIN);
      cst = map(vitesse,0,4095,0,270);
      cst1 = map(vitesse,0,4095,0,60);
      memset(serialbuffer,0,sizeof(serialbuffer));
      sprintf(serialbuffer,"digitalspeed.val=%d",cst1);
      hmiprint(serialbuffer);
      
      memset(serialbuffer,0,sizeof(serialbuffer));
      sprintf(serialbuffer,"Analogspeed.val=%d",cst);
      hmiprint(serialbuffer);
      if(vitesse>max_speed){
        max_speed=vitesse;
        cst2 = map(vitesse,0,4095,0,60);
      }
      memset(serialbuffer,0,sizeof(serialbuffer));
      sprintf(serialbuffer,"x0.val=%d",cst2);
      hmiprint(serialbuffer);
      tm1 = millis() + 500;
      

    /*calcul_v();
    cst = map(vit,0,60,0,270);

    memset(serialbuffer,0,sizeof(serialbuffer));
    sprintf(serialbuffer,"digitalspeed.val=%d",vit);
    hmiprint(serialbuffer);

    memset(serialbuffer,0,sizeof(serialbuffer));
    sprintf(serialbuffer,"Analogspeed.val=%d",cst);
    hmiprint(serialbuffer);
    if(vit>max_speed){
      max_speed=vit;
    }
    memset(serialbuffer,0,sizeof(serialbuffer));
    sprintf(serialbuffer,"x0.val=%d",max_speed);
    hmiprint(serialbuffer);
    tm1 = millis() + 500;*/
    }
  }

  if(CurrentPage == 4){  // If the display is on page 4, do the following:
  
  }

  if (controleurCAN.receive(messageCANReception1)) {
    //Un message CAN est arrive 
    controleurCAN.receive(messageCANReception1) ;
    phareState= messageCANReception1.data[0];
    distance2 = messageCANReception1.data[1];
    distance2 = 100-distance2;
    // l'etat de la LED correspondante est change 
    if(autoLight){
      digitalWrite(Front_light,phareState);
    }
  }  
}
