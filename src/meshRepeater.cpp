// Repeater for Mesh
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

Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

// Prototype so PlatformIO doesn't complain
/*void timer1sec();
void timer2sec();
void timer10sec();

Task task1sec(TASK_SECOND * 1, TASK_FOREVER, &timer1sec);
Task task2sec( TASK_SECOND * 2 , TASK_FOREVER, &timer2sec);
Task task10sec( TASK_SECOND * 10 , TASK_FOREVER, &timer10sec);

void timer1sec(){
}

void timer2sec(){
}

void timer10sec(){
}

void sendMessage()
{
  // Serializing in JSON Format
  DynamicJsonDocument doc(1024);
  String msg ;
  serializeJson(doc, msg);
  mesh.sendBroadcast(msg);
  Serial.println("Message sent: " + msg);
  //taskSendMessage.setInterval((TASK_SECOND * 1));
}
*/
void receivedCallback( uint32_t from, String &msg )
{
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

void setup() {
  Serial.begin(115200);
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
/*
  userScheduler.addTask( task1sec );
  task1sec.enable();
  userScheduler.addTask( task2sec );
  task2sec.enable();
  userScheduler.addTask( task10sec );
  task10sec.enable();
  */
}

void loop() {
  esp_task_wdt_reset();
  mesh.update();
}