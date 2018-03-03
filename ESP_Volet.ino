

/* Version du 21/01/2018 par vincnet68

 * SKETCH fonctionnant sur wemos D1 MINI POUR COMMANDE DE VOLET ROULANT FILAIRE AVEC RENVOI DE LA POSITION DU VOLET EN POURCENTAGE
 * HARD Module sur Rail DIN (Alcor_fr et Rolrider): https://www.jeedom.com/forum/viewtopic.php?f=185&t=25017&sid=c757bad46d600f07820dab2a45ec8b33
 * LIBRAIRIES : https://github.com/marvinroger/arduino-shutters
 *              https://github.com/mathertel/OneButton
 *              https://github.com/esp8266/Arduino/blob/master/libraries/Ticker/Ticker.h
 * AJOUT de l'OTA 
 * AJOUT de WiFiManager
 * Possibilité d'enregistrer l'adresse IP de son broker MQTT
 * Possibilité d'enregistrer le temp de course
 * Possibilité d'enregistrer le nom du module (utilisé pour le WifiManager portal, et la publication MQTT)
 * Utilisation de 2 boutons poussoir pour la commande (commande local, et envoie de double click et long click par MQTT)
 * Utilisation de la lib Tick pour garder les commandes local disponible même en cas de déconnection wifi ou MQTT
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
#include <OneButton.h>
#include <Ticker.h>

bool DisableRazFunction = false; //Disable the raz function with the button
bool debug = true;  //Affiche sur la console si True
bool raz = false;   //Réinitialise la zone SPIFFS et WiFiManager si True
long lastMsg = 0;
long lastConnect = 0;
char mqtthost[16] = ""; //Variable qui sera utilisée par WiFiManager pour enregistrer l'adresse IP du broker MQTT
const char* PASS = ""; //A modifier avec le mot de passe voulu, il sera utilise pour les mises a jour OTA
bool isMoving = false; //Indique si les volets sont en mouvement
char timeCourseup[3] = ""; //Variable qui sera utilisée par WiFiManager pour enregistrer le temp de course du volet
char timeCoursedown[3] = "";
const byte eepromOffset = 0;
unsigned long upCourseTime = 20 * 1000; //Valeur par défaut du temps de course en montée, un temps de course de descente peut aussi être défini
unsigned long downCourseTime = 20 * 1000;
const float calibrationRatio = 0.1;
char ESP8266Client[20]  = "VR_Empty"; //Default
int doubleLongPressStart = 0; //Time during double long press
Ticker ticker;
bool localModeOnly = true; //Disable the mqtt send while it is not connected
bool shutterInitialized = false; //Indicate if the shutter library has been initialized
//Wifimanager 
         
//flag for saving data
bool shouldSaveConfig = false;
//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
  localModeOnly = false;
  ticker.detach();
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  localModeOnly = true;
  ticker.attach(0.05, loopLocalShutter);
}


// NOMAGE MQTT
#define relais1_topic "/Relais1"
#define relais2_topic "/Relais2"
//#define entree1_topic "/Entree1"
//#define entree2_topic "/Entree2"
#define position_topic "/Position"
#define BtUpClick_topic "/BtUpClick"
#define BtUpLongClick_topic "/BtUpLongClick"
#define BtUpDblClick_topic "/BtUpDblClick"
#define BtDownClick_topic "/BtDownClick"
#define BtDownLongClick_topic "/BtDownLongClick"
#define BtDownDblClick_topic "/BtDownDblClick"
#define JeedomIn_topic "/in"
#define JeedomOut_topic "/out"
#define prefix_topic "Jeedom/"
#define CONST_TRUE "1"
#define CONST_FALSE "0"

// DEFINITION DES GPIOS
//RELAIS 1
const int R1Pin = 4;
//RELAIS 2
const int R2pin = 5;
//ENTREE 1
const int In1pin = 14;
//ENTREE 2
const int In2pin = 12;
// Setup a new OneButton 
OneButton button1(In1pin, false);
// Setup a new OneButton
OneButton button2(In2pin, false);

// VARIABLES
//POSITION
const char* cmdnamePos = "Position";

char message_buff[100];

WiFiClient espClientVR_Test;  // A renommer pour chaque volets
PubSubClient client(espClientVR_Test); // A renommer pour chaque volets

//***********************************************************************************
// FONCTIONS LIBRAIRIE position volets

void shuttersOperationHandler(Shutters* s, ShuttersOperation operation) {
  switch (operation) {
    case ShuttersOperation::UP:
      if (debug){Serial.println("Shutters going up.");}
        up();       
      break;
    case ShuttersOperation::DOWN:
  if (debug){Serial.println("Shutters going down.");}
        dwn();
      break;
    case ShuttersOperation::HALT:
      if (debug){Serial.println("Shutters halting.");}
        stp();
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
  char charlevel[4];
  sprintf(charlevel, "%d", level); 
  mqttPublish(position_topic,charlevel); 
  }
}

Shutters shutters;
//

//***********************************************************************************
// SETUP

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);   
  digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on to indicate the the module is starting

// INITIALYZE GPIO
  pinMode(R1Pin, OUTPUT);
  pinMode(R2pin, OUTPUT);
  //digitalWrite(R1Pin, 0);
  //digitalWrite(R2pin, 0);

  // link the button 1 functions.
  button1.attachClick(click1);
  button1.attachDoubleClick(doubleclick1);
  button1.attachLongPressStart(longPressStart1);
  button1.attachLongPressStop(longPressStop1);
  button1.attachDuringLongPress(longPress1);

  // link the button 2 functions.
  button2.attachClick(click2);
  button2.attachDoubleClick(doubleclick2);
  button2.attachLongPressStart(longPressStart2);
  button2.attachLongPressStop(longPressStop2);
  button2.attachDuringLongPress(longPress2);

  
  localModeOnly = true;
  ticker.attach(0.05, loopLocalShutter);
  //SERIAL//
  Serial.begin(115200);
  delay(100);
  #ifdef ESP8266
  EEPROM.begin(512);
  #endif
  Serial.println();
  Serial.println("*** Starting ***");
  Serial.println(ESP8266Client);                

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
          strcpy(ESP8266Client, json["ESP8266Client"]);
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
  wifiManager.setAPCallback(configModeCallback);
  //Ajout de la variable de configuration MQTT Server (ou Broker)
  WiFiManagerParameter custom_mqtthost("server", "mqtt server", mqtthost, 16);
  WiFiManagerParameter custom_timeCourse_up("time_course_up", "Time Course Up", timeCourseup, 3);
  WiFiManagerParameter custom_timeCourse_down("time_course_down", "Time Course Down", timeCoursedown, 3);
  WiFiManagerParameter custom_ESP8266Client("ESP8266Client", "ESPName", ESP8266Client, 20);
  wifiManager.addParameter(&custom_mqtthost);
  wifiManager.addParameter(&custom_timeCourse_up);
  wifiManager.addParameter(&custom_timeCourse_down);
  wifiManager.addParameter(&custom_ESP8266Client);

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
  strcpy (ESP8266Client, custom_ESP8266Client.getValue());
  //Sauvegarde des valeurs dans le fichier de configuration json
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtthost"] = mqtthost;
    json["timeCourseup"] = timeCourseup;
    json["timeCoursedown"] = timeCoursedown;
    json["ESP8266Client"] = ESP8266Client;
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("Erreur lors de l'ouverture du fichier de config json pour enregistrement");
    }
    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

//Shutters //
	stp();//Stop the shutter before initialize
 char storedShuttersState[shutters.getStateLength()];
  readInEeprom(storedShuttersState, shutters.getStateLength());
  shutters
    .setOperationHandler(shuttersOperationHandler)
    .setWriteStateHandler(shuttersWriteStateHandler)
    .restoreState(storedShuttersState)
    .setCourseTime(upCourseTime, downCourseTime)
    .onLevelReached(onShuttersLevelReached)
    .begin();
  shutterInitialized = true;
//MQTT//
  
  client.setServer(mqtthost, 1883);
  Serial.print("host MQTT :");
  Serial.println(mqtthost);
  client.setCallback(callback);


  Serial.print("storedShuttersState :"); 
  Serial.println(storedShuttersState);  

  Serial.println("Shutter Begin");
  mqttInit();
  digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED off to indicate the the module is Ready
  localModeOnly = false;
  ticker.detach();
}



//***********************************************************************************
// FONCTION Reset + Format memoire ESP
void eraz(){
  WiFiManager wifiManager;
if (debug){Serial.println("Réinitialisation de WiFiManager.");}
  wifiManager.resetSettings();
if (debug){Serial.println("Réinitialisation de la configuration (reset SPIFFS).");}
  //SPIFFS.format();
ESP.restart();
}

void shutterRaz(){
  if (debug){Serial.println("Réinitialisation shutter");}
  shutters
    .reset()
    .setOperationHandler(shuttersOperationHandler)
    .setWriteStateHandler(shuttersWriteStateHandler)
    .setCourseTime(upCourseTime, downCourseTime)
    .onLevelReached(onShuttersLevelReached)
    .begin();
}

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
   
  if (niv >= 0 && niv <= 100 &&  msgString != "up" && msgString != "dwn" && msgString != "stp" && msgString != "raz" && msgString != "shutraz"){
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
  } else if ( msgString == "shutraz" ){
  shutterRaz();
  }
}

//***********************************************************************************
//Fonction reconnexion MQTT
void reconnect() { 
  // Loop until we're reconnected
  long now1 = millis();
  if (debug){    
    Serial.print("Attente de connexion MQTT...");}
	digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on to indicate the the module is not Ready
	ticker.attach(0.05, loopLocalShutter);
    // Attempt to connect
    if (client.connect(ESP8266Client)) {  
      float connectedepuis = (now1 - lastConnect)/1000;
      if (debug){Serial.print("connected depuis :");
      Serial.println(connectedepuis);}
      // Once connected, publish an announcement...
      String topicString = String();
      topicString = String(prefix_topic) + String(ESP8266Client) + String(JeedomIn_topic);
      client.publish(string2char(topicString), String(connectedepuis).c_str());
      lastConnect = now1;
      // ... and resubscribe
      topicString = String(prefix_topic) + String(ESP8266Client) + String(JeedomOut_topic);
      client.subscribe(string2char(topicString));
	  digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED off to indicate the the module is Ready	
	  ticker.detach();
    } else {
  if (debug){    
      Serial.print("Erreur, rc=");
      Serial.print(client.state());}               
    }
  }

//***********************************************************************************
// LOOP
void loop(void){
  unsigned long currentMillis = millis();
  unsigned long now = millis();
  //Serial.println(" now :");
  //Serial.println(now);

  if (now - lastMsg > 5000) {
    //Serial.println(" now :");
    //Serial.println(now);
    lastMsg = now;
    if (!client.connected()) {
      if (debug){Serial.println("client reconnexion");}
      reconnect();       
    }         
  }
  ArduinoOTA.handle();
  client.loop();
  loopLocalShutter();
}

//Local shutter management
void loopLocalShutter()
{
  if (shutterInitialized) shutters.loop();  
  button1.tick();
  button2.tick();
  
   //Detect the longpress on both button
  if (!DisableRazFunction && button1.isLongPressed() && button2.isLongPressed())
  {
    doubleLongPressStart = millis() - doubleLongPressStart;
    if (doubleLongPressStart > 10*1000) //More the 10 second long Press
    {
      shutters.stop();
     eraz();
    }
 }
  else
    doubleLongPressStart = millis();
}

//***********************************************************************************
// FONCTION MQTT ESP VERS JEEDOM
void mqttPublish(char* topic, char* value)
{  
  if (!localModeOnly){
     if (!client.connected()) {
      reconnect();
    }
    else {  
      if (debug){
      Serial.print("envoi MQTT");
    Serial.print(topic);
    Serial.print(" : ");
    Serial.println(value);
      }
     String topicString = String();
      topicString = String(prefix_topic) + String(ESP8266Client) + String(topic);
      client.publish(string2char(topicString), value);
    }
  }
}
void mqttInit(){
  char level[4];
  int levelInt;
  levelInt =  shutters.getCurrentLevel();
  
  sprintf(level, "%d", levelInt); 
    if (debug){
    Serial.println("envoi MQTT d'initialisation");
    }
    mqttPublish(position_topic, level);
    mqttPublish(BtUpClick_topic,CONST_FALSE);
    mqttPublish(BtUpLongClick_topic,CONST_FALSE);
    mqttPublish(BtUpDblClick_topic,CONST_FALSE);
    mqttPublish(BtDownClick_topic,CONST_FALSE);
    mqttPublish(BtDownLongClick_topic,CONST_FALSE);
    mqttPublish(BtDownDblClick_topic,CONST_FALSE);

}

//***********************************************************************************
// FONCTIONS MOUVEMENTS VOLET
void up(){
  if (debug) {Serial.println("Action up");}
  digitalWrite(R1Pin, 1);
  digitalWrite(R2pin, 0);
  isMoving = true;
  if (!localModeOnly)
  {
    mqttPublish(relais1_topic, CONST_TRUE);
  }
 }

void dwn(){
  if (debug) {Serial.println("Action down");}
  digitalWrite(R1Pin, 0);
  digitalWrite(R2pin, 1);
  isMoving = true;
    if (!localModeOnly)
  {
    mqttPublish(relais2_topic, CONST_TRUE);
  }
  }

void stp(){
  if (debug) {Serial.println("Action stop");}
  digitalWrite(R1Pin, 0);
  digitalWrite(R2pin, 0);
  isMoving = false;
  char level[4];
  int levelInt;
  if (!localModeOnly) 
  {
      levelInt =  shutters.getCurrentLevel();
      sprintf(level, "%d", levelInt); 
      mqttPublish(position_topic, level);
      mqttPublish(relais1_topic, CONST_FALSE);
      mqttPublish(relais2_topic, CONST_FALSE);
      
  }
  
  }
  

//***********************************************************************************
//FONCTION SORTIES RELAIS EN FONCTION DES ENTREES POUR COMMANDE LOCAL
// ----- button 1 callback functions
// This function will be called when the button1 was pressed 1 time (and no 2. button press followed).
void click1() {

 if (isMoving){
    if (debug){
      Serial.println("Click Up Shutter Moving" );    
    }
      if (!shutterInitialized) stp();
      else shutters.stop();
    }
   else{
     if (debug){
          Serial.println("Click Up Shutter Up" );
     }
              if (shutters.getCurrentLevel() != 100){
    
      if (!shutterInitialized) up();
      else shutters.setLevel(100);
     }
   }
  mqttPublish(BtUpClick_topic, CONST_TRUE);
  mqttPublish(BtUpClick_topic, CONST_FALSE);
} // click1

// This function will be called when the button1 was pressed 2 times in a short timeframe.
void doubleclick1() {
  Serial.println("Button 1 doubleclick.");
    mqttPublish(BtUpDblClick_topic,CONST_TRUE); 
    mqttPublish(BtUpDblClick_topic,CONST_FALSE); 
} // doubleclick1

// This function will be called once, when the button1 is pressed for a long time.
void longPressStart1() {
  Serial.println("Button 1 longPress start");
  mqttPublish(BtUpLongClick_topic,CONST_TRUE); 
  mqttPublish(BtUpLongClick_topic,CONST_FALSE); 
} // longPressStart1

// This function will be called often, while the button1 is pressed for a long time.
void longPress1() {
  Serial.println("Button 1 longPress...");
} // longPress1

// This function will be called once, when the button1 is released after beeing pressed for a long time.
void longPressStop1() {
  Serial.println("Button 1 longPress stop");
} // longPressStop1

// ... and the same for button 2:

void click2() {
 if (isMoving){
    if (debug){
                 
      Serial.println("Click Down Shutter Moving" );    ;
    }
      if (!shutterInitialized) stp();
      else shutters.stop();
    }
   else{
 if (debug){
      Serial.println("Click Down Shutter Down" );    ;
                 
    }
          if (shutters.getCurrentLevel() != 0){
      if (!shutterInitialized) dwn();
      else shutters.setLevel(0);
   }
   }
   mqttPublish(BtDownClick_topic,CONST_TRUE); 
   mqttPublish(BtDownClick_topic,CONST_FALSE); 
} // click2

void doubleclick2() {
  Serial.println("Button 2 doubleclick.");
  mqttPublish(BtDownDblClick_topic,CONST_TRUE); 
  mqttPublish(BtDownDblClick_topic,CONST_FALSE); 
} // doubleclick2

void longPressStart2() {
  Serial.println("Button 2 longPress start");
  mqttPublish(BtDownLongClick_topic,CONST_TRUE); 
  mqttPublish(BtDownLongClick_topic,CONST_FALSE); 
} // longPressStart2

void longPress2() {
  Serial.println("Button 2 longPress...");
} // longPress2



void longPressStop2() {

  Serial.println("Button 2 longPress stop");
} // longPressStop2


char* string2char(String command){
    if(command.length()!=0){
        char *p = const_cast<char*>(command.c_str());
        return p;
    }

}
