//
//************************************************************

#include <Arduino.h>
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 100
// Necessary Libraries
#include <painlessMesh.h>
#include <ArduinoJson.h>

// WiFi Credentials
#define   MESH_PREFIX     "ESP_Mesh"
#define   MESH_PASSWORD   "somethingSneaky10"
#define   MESH_PORT       5555

//Pin Declaration
#define soilhumidityPin 36
#define irrigationValve5Pin 4
#define irrigationValve6Pin 16
#define irrigationValve7Pin 17
#define irrigationValve8Pin 18


//Variables
int irrigationValve;
bool valve5;
bool valve6;
bool valve7;
bool valve8;
int lastHeardFromUART = 0;
float soilHumidity = 0;
int soilHumidityArray[100];

Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

void sendMessage(); // Prototype so PlatformIO doesn't complain
void timer1sec();
void timer2sec();
void timer10sec();
void turnValvesOnOff();
void measureSoilHumidity();
void sendMessage();

Task task1sec(TASK_SECOND * 1, TASK_FOREVER, &timer1sec);
Task task2sec(TASK_SECOND * 2 , TASK_FOREVER, &timer2sec);
Task task10sec(TASK_SECOND * 10 , TASK_FOREVER, &timer10sec);

void turnValvesOnOff(){
  esp_task_wdt_reset();
//  if (lastHeardFromUART+(1000*60*30) < millis()) irrigationValve = 0; // Turn off irrigation if connection has been lost for 30 min
  valve5=false;
  valve6=false;
  valve7=false;
  valve8=false;
  switch(irrigationValve){
    case 5:
    valve5=true;
    break;
    case 6:
    valve6=true;
    break;
    case 7:
    valve7=true;
    break;
    case 8:
    valve8=true;
    break;
  }
  digitalWrite(irrigationValve5Pin, valve5);
  digitalWrite(irrigationValve6Pin, valve6);
  digitalWrite(irrigationValve7Pin, valve7);
  digitalWrite(irrigationValve8Pin, valve8);
  Serial.print("irrigationValve5: ");
  Serial.println(valve5);
  Serial.print("irrigationValve6: ");
  Serial.println(valve6);
  Serial.print("irrigationValve7: ");
  Serial.println(valve7);
  Serial.print("irrigationValve8: ");
  Serial.println(valve8);
}

void measureSoilHumidity(){ // avg 100 samples and MA100
  esp_task_wdt_reset();
  int soilHumiditySample = 0;
  int soilHumiditySum = 0;
  for (int i=0; i < 100; i++){ // avg of 100 samples
    soilHumiditySample += map(analogRead(soilhumidityPin), 1700, 2500, 100, 0) ;
  }
  soilHumiditySample = soilHumiditySample/100;
  for(int i=0;i<99;i++){ // move stored values to the previous record 
    soilHumidityArray[i] = soilHumidityArray[i+1];
    soilHumiditySum += soilHumidityArray[i];
  }
  soilHumidityArray[99] = soilHumiditySample;
  soilHumiditySum += soilHumiditySample;
  soilHumidity = soilHumiditySum / 100.00;
}

  
void sendMessage(){
  esp_task_wdt_reset();
  // Serializing in JSON Format
  DynamicJsonDocument doc(1024);
  doc["soilHumidity"] = soilHumidity;
  doc["upTimeShed"] = millis()/1000.00/60.00/60.00;
  doc["irrigationValveShed"] = irrigationValve; // checking to see what is returned from shed
  String msg ;
  serializeJson(doc, msg);
  mesh.sendBroadcast(msg);
  Serial.println("Message sent: " + msg);
  //taskSendMessage.setInterval((TASK_SECOND * 1));
}

// Needed for painless library
void receivedCallback( uint32_t from, String &msg ){
  lastHeardFromUART = millis();
  //Deserializing
  String json;
  DynamicJsonDocument doc(1024);
  json = msg.c_str();
  Serial.println("Received message: " + msg + " From: " + from + " Json: " + json);
  DeserializationError error = deserializeJson(doc, json);
  
  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
  }
  irrigationValve = doc["IrrigationValve"];
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void timer1sec(){
  esp_task_wdt_reset();
  measureSoilHumidity();
}

void timer2sec(){
}

void timer10sec(){
  esp_task_wdt_reset();
  turnValvesOnOff();
  sendMessage();
}

void setup() {
  Serial.begin(115200);

  pinMode(irrigationValve5Pin, OUTPUT);
  pinMode(irrigationValve6Pin, OUTPUT);
  pinMode(irrigationValve7Pin, OUTPUT);
  pinMode(irrigationValve8Pin, OUTPUT);
  
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  userScheduler.addTask( task1sec );
  task1sec.enable();
  userScheduler.addTask( task2sec );
  task2sec.enable();
  userScheduler.addTask( task10sec );
  task10sec.enable();
}

void loop() {
  esp_task_wdt_reset();
  // it will run the user scheduler as well
  mesh.update();
}