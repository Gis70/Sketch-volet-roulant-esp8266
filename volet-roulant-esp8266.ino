/* Release du 01/01/2018 pour module electrodragon
 * SKETCH fonctionnant sur wemos D1 MINI POUR COMMANDE DE VOLET ROULANT FILAIRE AVEC RENVOI DE LA POSITION DU VOLET EN POURCENTAGE
 * HARD : https://www.jeedom.com/forum/viewtopic.php?f=185&t=25017&sid=c757bad46d600f07820dab2a45ec8b33
 * LIBRAIRIES : https://github.com/marvinroger/arduino-shutters V3 beta4 (voir bibliothèque IDE Arduino)
 * AJOUT de l'OTA 
 * AJOUT de WiFiManager
 * Possibilité d'enregistrer l'adresse IP de son broker MQTT
 * Possibilité d'enregistrer les temps de course monté et descente
 */
#include <FS.h>
#include <Shutters.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <EEPROM.h>


bool debug = true;  //Affiche sur la console si True
bool raz = false;   //Réinitialise la zone SPIFFS et WiFiManager si True
long lastMsg = 0;
long lastConnect = 0;
char mqtthost[16] = ""; //Variable qui sera utilisée par WiFiManager pour enregistrer l'adresse IP du broker MQTT
const char* PASS = "password"; //A modifier avec le mot de passe voulu, il sera utilise pour les mises a jour OTA

char timeCourseup[3] = ""; //Variable qui sera utilisée par WiFiManager pour enregistrer le temp de course du volet
char timeCoursedown[3] = "";
const byte eepromOffset = 0;
unsigned long upCourseTime = 20 * 1000; //Valeur par défaut du temps de course en montée, un temps de course de descente peut aussi être défini
unsigned long downCourseTime = 20 * 1000;
const float calibrationRatio = 0.1;

int R1state;  // remove init value
int R2state;
int In1state;
int In2state;
int lastR1state;
int lastR2state;
int lastIn1state;
int lastIn2state;

//Wifimanager 
//flag for saving data
bool shouldSaveConfig = false;
//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


// NOMAGE MQTT
#define relais1_topic "Volet/PFSalon/Relais1"
#define relais2_topic "Volet/PFSalon/Relais2"
#define entree1_topic "Volet/PFSalon/Entree1"
#define entree2_topic "Volet/PFSalon/Entree2"
#define position_topic "Volet/PFSalon/Position"
#define JeedomIn_topic "Volet/PFSalon/in"
#define JeedomOut_topic "Volet/PFSalon/out"
#define ESP8266Client "Volet_PFSalon" //Nom du peripherique sur le reseau

//RELAIS 1    peut être doit on les inverser
const int R1pin = 13; // Relais Up
//RELAIS 2
const int R2pin = 12;  // Relais Down
//ENTREE 1
const int In1pin = 5; // Up
//ENTREE 2
const int In2pin = 4; // Down

// VARIABLES

char message_buff[100];

WiFiClient espClientVolet_PFSalon;  // A renommer pour chaque volets
PubSubClient client(espClientVolet_PFSalon); // A renommer pour chaque volets

//***********************************************************************************
// FONCTIONS LIBRAIRIE position volets

void shuttersOperationHandler(Shutters* s, ShuttersOperation operation) {
  switch (operation) {
    case ShuttersOperation::UP:
      if (debug){Serial.println("Shutters going up.");}
        up();
        //mqtt();
      break;
    case ShuttersOperation::DOWN:
      if (debug){Serial.println("Shutters going down.");}
        dwn();
        //mqtt();
      break;
    case ShuttersOperation::HALT:
      if (debug){Serial.println("Shutters halting.");}
        stp();
        mqttlevel();
      break;
  }
}

void readInEeprom(char* dest, byte length) {
  for (byte i = 0; i < length; i++) {
    dest[i] = EEPROM.read(eepromOffset + i);
  }
}

void shuttersWriteStateHandler(Shutters* shutters, const char* state, byte length) {
  for (byte i = 0; i < length; i++) {
    EEPROM.write(eepromOffset + i, state[i]);
    #ifdef ESP8266
    EEPROM.commit();
    #endif
  }
}

void onShuttersLevelReached(Shutters* shutters, byte level) {
  if (debug){
  Serial.print("Shutters at ");
  Serial.print(level);
  Serial.println("%");
  }
  if ((level % 10) == 0) {
  mqttlevel();  
  }
  }

Shutters shutters;
//

//***********************************************************************************
// SETUP

void setup() {
  
  
  //SERIAL//
  Serial.begin(115200);
  delay(100);
  #ifdef ESP8266
  EEPROM.begin(512);
  #endif
  Serial.println();
  Serial.println("*** Starting ***");

  Serial.println(ESP8266Client);
  
  R1state = digitalRead(R1pin);
  R2state = digitalRead(R2pin);
  In1state = digitalRead(In1pin);
  In2state = digitalRead(In2pin);
  lastR1state = R1state;
  lastR2state = R2state;
  lastIn1state = In1state;
  lastIn2state = In2state;
  if (debug){
  Serial.print("R1state:" );
  Serial.println(R1state); 
  Serial.print("R2state:" );
  Serial.println(R2state); 
  Serial.print("In1state:" );
  Serial.println(In1state); 
  Serial.print("In2state:" );
  Serial.println(In2state); 
  }
     
if (raz){
  Serial.println("Réinitialisation de la configuration (reset SPIFFS).");
  SPIFFS.format();
 }
  //Lecture du fichier de configuration depuis le FS avec json
  Serial.println("montage du FS...");
  if (SPIFFS.begin()) {
    Serial.println("FS monté");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("Lecture du fichier de config");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("Fichier de config ouvert");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nJson parsé");
          strcpy(mqtthost, json["mqtthost"]);
          strcpy(timeCourseup, json["timeCourseup"]);
          strcpy(timeCoursedown, json["timeCoursedown"]);
          upCourseTime = (strtoul (timeCourseup, NULL, 10)) * 1000; //On retype la variable (%ul unsigned long) et on la multiplie par 1000 (ce sont des millisecondes)
          downCourseTime = (strtoul (timeCoursedown, NULL, 10)) * 1000;
         } else {
          Serial.println("Erreur lors du chargement du fichier de config json");
        }
      }
    }
  } else {
    Serial.println("Erreur lors du montage du FS");
  }
  //end read
  
  //WIFI// 
  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  //Ajout de la variable de configuration MQTT Server (ou Broker)
  WiFiManagerParameter custom_mqtthost("server", "mqtt server", mqtthost, 16);
  WiFiManagerParameter custom_timeCourse_up("time_course_up", "Time Course Up", timeCourseup, 3);
  WiFiManagerParameter custom_timeCourse_down("time_course_down", "Time Course Down", timeCoursedown, 3);
  wifiManager.addParameter(&custom_mqtthost);
  wifiManager.addParameter(&custom_timeCourse_up);
  wifiManager.addParameter(&custom_timeCourse_down);
  
  wifiManager.autoConnect(ESP8266Client, PASS);

//reset settings - for testing
if (raz){
  Serial.println("Réinitialisation de WiFiManager.");
  wifiManager.resetSettings();
  }
// Configuration OTA
  //Port 8266 (defaut)
  ArduinoOTA.setPort(8266);
  //Hostname 
  ArduinoOTA.setHostname(ESP8266Client);
  //Mot de passe
  ArduinoOTA.setPassword(PASS);
  //Depart de la mise a jour
  ArduinoOTA.onStart([]() {
    Serial.println("Maj OTA");
  });
  //Fin de la mise a jour
  ArduinoOTA.onEnd([]() {
    Serial.print("\n Maj terminee");
  });
  //Pendant la mise a jour
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progression : %u%%\r", (progress / (total / 100)));
  });
  //En cas d'erreur
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Erreur[%u]: ", error);
    switch (error) {
      //Erreur d'authentification
      case OTA_AUTH_ERROR :     Serial.println("Erreur d'authentification lors de la mise à jour");
                                break;
      case OTA_BEGIN_ERROR :    Serial.println("Erreur lors du lancement de la mise à jour");
                                break;
      case OTA_CONNECT_ERROR :  Serial.println("Erreur de connexion lors de la mise à jour");
                                break;
      case OTA_RECEIVE_ERROR :  Serial.println("Erreur de reception lors de la mise à jour");
                                break;
      case OTA_END_ERROR :      Serial.println("Erreur lors de la phase finale de la mise à jour");
                                break;
      default:                  Serial.println("Erreur inconnue lors de la mise à jour");
    }
  });
  ArduinoOTA.begin();
  //Fin conf OTA
  
  // on affiche l'adresse IP qui nous a été attribuée
  Serial.println("");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  //Lecture de la valeur MQTT server enregistrée dans le fichier de config json
  strcpy(mqtthost, custom_mqtthost.getValue());
  strcpy(timeCourseup, custom_timeCourse_up.getValue());
  upCourseTime = (strtoul (timeCourseup, NULL, 10)) * 1000; //On retype la variable (%ul unsigned long) et on la multiplie par 1000 (ce sont des millisecondes)
  strcpy(timeCoursedown, custom_timeCourse_down.getValue());
  downCourseTime = (strtoul (timeCoursedown, NULL, 10)) * 1000;
  //Sauvegarde des valeurs dans le fichier de configuration json
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtthost"] = mqtthost;
    json["timeCourseup"] = timeCourseup;
    json["timeCoursedown"] = timeCoursedown;
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("Erreur lors de l'ouverture du fichier de config json pour enregistrement");
    }
    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
//MQTT//
  
  client.setServer(mqtthost, 1883);
  Serial.print("host MQTT :");
  Serial.println(mqtthost);
  client.setCallback(callback);

// INITIALYZE GPIO
  pinMode(R1pin, OUTPUT);
  pinMode(R2pin, OUTPUT);
  //digitalWrite(R1pin, 0);
  //digitalWrite(R2pin, 0);
  pinMode(In1pin, INPUT_PULLUP);
  pinMode(In2pin, INPUT_PULLUP);

    
  char storedShuttersState[shutters.getStateLength()];
  readInEeprom(storedShuttersState, shutters.getStateLength());
  shutters
    .setOperationHandler(shuttersOperationHandler)
    .setWriteStateHandler(shuttersWriteStateHandler)
    .restoreState(storedShuttersState)
    .setCourseTime(upCourseTime, downCourseTime)
    .onLevelReached(onShuttersLevelReached)
    .begin();
  
  if (debug){Serial.println("Shutter Begin");
  }
}
//***********************************************************************************
// FONCTION Reset + Format memoire ESP
void eraz(){
  WiFiManager wifiManager;
if (debug){Serial.println("Réinitialisation de WiFiManager.");}
  wifiManager.resetSettings();
if (debug){Serial.println("Réinitialisation de la configuration (reset SPIFFS).");}
  SPIFFS.format();
ESP.reset();
}

//***********************************************************************************
// FONCTION communication MQTT JEEDOM VERS ESP
void callback(char* topic, byte* payload, unsigned int length) {
  int i = 0;
  if ( debug ) {
    Serial.println("Message recu =>  topic: " + String(topic));
    Serial.println(" | longueur: " + String(length,DEC));
  }
  // create character buffer with ending null terminator (string)
  for(i=0; i<length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  
  String msgString = String(message_buff);
  if ( debug ) {
    Serial.println("Payload: " + msgString);
  }
   uint8_t niv = msgString.toInt();
   
  if (niv >= 0 && niv <= 100 &&  msgString != "up" && msgString != "dwn" && msgString != "stp" && msgString != "raz"){
    shutters.setLevel(niv);
  }
    else if ( msgString == "up" ) {
    shutters.setLevel(100); 
  } else if ( msgString == "dwn" ){
    shutters.setLevel(0);  
  } else if ( msgString == "stp" ){
    shutters.stop();  
  } else if ( msgString == "raz" ){
    eraz();
  }
}

//***********************************************************************************
//Fonction reconnexion MQTT
void reconnect() { 
  // Loop until we're reconnected
  long now1 = millis();
  if (debug){
  Serial.print("Attente de connexion MQTT...");}
    // Attempt to connect
    if (client.connect(ESP8266Client)) {
      float connectedepuis = (now1 - lastConnect)/1000;
      if (debug){Serial.print("connected depuis :");
      Serial.println(connectedepuis);}
      // Once connected, publish an announcement...
      client.publish(JeedomIn_topic, String(connectedepuis).c_str());
      lastConnect = now1;
      // ... and resubscribe
      client.subscribe(JeedomOut_topic);
    } else {
  if (debug){
      Serial.print("Erreur, rc=");
      Serial.print(client.state());
  Serial.println(" Nouvelle tentative dans 5 secondes");}
      // Wait 5 seconds before retrying
      }
  }

//***********************************************************************************
// LOOP
void loop(void){
  
R1state = digitalRead(R1pin);
R2state = digitalRead(R2pin);
In1state = digitalRead(In1pin);
In2state = digitalRead(In2pin);

  unsigned long now = millis();
  
  if (now - lastMsg > 5000) {
     lastMsg = now;
    if (!client.connected()) {
      if (debug){Serial.println("client reconnexion");}
      reconnect();
    }}
  
if (R1state != lastR1state){
  if (debug){Serial.println("declenchement par R1" );}
  mqttR1();
  lastR1state = R1state;
}
if (R2state != lastR2state){
  if (debug){Serial.println("declenchement par R2" );}
  mqttR2();
  lastR2state = R2state;
} 
if (In1state != lastIn1state){
  if (debug){Serial.println("declenchement par In1" );}
  OutIn();
  mqttIn1();
  lastIn1state = In1state;
}
if (In2state != lastIn2state){
  if (debug){Serial.println("declenchement par In2" );}
  OutIn();
  mqttIn2();
  lastIn2state = In2state;
}
  ArduinoOTA.handle();
  client.loop();
  shutters.loop();  
}

//***********************************************************************************
// FONCTION MQTT ESP VERS JEEDOM
void mqttR1(){
char Relais1[2];
sprintf(Relais1, "%d", digitalRead(R1pin)); 
if (!client.connected()) {
    reconnect();
  }
  else {
    if (debug){
   Serial.print("envoi MQTTR1 :");
   Serial.println(Relais1);
 }
client.publish(relais1_topic, Relais1);
}
}

void mqttR2(){
char Relais2[2];
sprintf(Relais2, "%d", digitalRead(R2pin));
if (!client.connected()) {
    reconnect();
  }
  else {
    if (debug){
   Serial.print("envoi MQTTR2 :");
   Serial.println(Relais2);
 }
client.publish(relais2_topic, Relais2);
}
}

void mqttIn1(){
char Entree1[2];
sprintf(Entree1, "%d", digitalRead(In1pin));
if (!client.connected()) {
    reconnect();
  }
  else {
if (debug){
   Serial.print("envoi MQTTIn1 :");
   Serial.println(Entree1);
 }
client.publish(entree1_topic, Entree1);
}
}

void mqttIn2(){
char Entree2[2];
sprintf(Entree2, "%d", digitalRead(In2pin));
if (!client.connected()) {
    reconnect();
  }
  else {
if (debug){
   Serial.print("envoi MQTTIn2 :");
   Serial.println(Entree2);
 }
client.publish(entree2_topic, Entree2);
}
}

void mqttlevel(){
  char level[4];
  sprintf(level, "%d", shutters.getCurrentLevel());
if (!client.connected()) {
    reconnect();
  }
  else {  
if (debug){
   Serial.print("envoi MQTTlevel :");
   Serial.println(level);
 }
     client.publish(position_topic, level);
 }
}

//***********************************************************************************
// FONCTIONS MOUVEMENTS VOLET
void up(){
  digitalWrite(R1pin, 1);
  digitalWrite(R2pin, 0);
  if (debug){Serial.println("MOUVEMENT: up" );}
  
  }

void dwn(){
  digitalWrite(R1pin, 0);
  digitalWrite(R2pin, 1);
  if (debug){Serial.println("MOUVEMENT: dwn" );}
  }

void stp(){
  digitalWrite(R1pin, 0);
  digitalWrite(R2pin, 0);
  if (debug){Serial.println("MOUVEMENT: stp" );}
  }

//***********************************************************************************
//FONCTION SORTIES RELAIS EN FONCTION DES ENTREES POUR COMMANDE LOCAL
void OutIn(){
    
int status = In1state*10 + In2state;
// status = 10 pour up, 1 pour dwn, 0 pour stp
int level1 = shutters.getCurrentLevel();

switch (status) { 
   case 10:
   
    if (debug){
      Serial.print("In1state:" );
      Serial.println(In1state);  
      Serial.print("In2state:" );
      Serial.println(In2state);  
      Serial.print("status:" );
      Serial.println(status);    
      }
      if (level1 != 100){
      shutters.setLevel(100);
      }
      
  break;


case 1:
   if (debug){
      Serial.print("In1state:" );
      Serial.println(In1state);  
      Serial.print("In2state:" );
      Serial.println(In2state);  
      Serial.print("status:" );
      Serial.println(status);    
      }
      if (level1 != 0){
      shutters.setLevel(0);
      }
   break;


case 11:
   
   shutters.stop();
   //mqtt(); 
   if (debug){
      Serial.print("In1state:" );
      Serial.println(In1state);  
      Serial.print("In2state:" );
      Serial.println(In2state);  
      Serial.print("status:" );
      Serial.println(status);    
      }
   
      
   break;

case 0:
break;

default: 
   if (debug){Serial.println("Erreur Switch Case");}
   break;

}  
}
