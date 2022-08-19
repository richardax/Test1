/* changes to m´be made
check if avgsample is decimal, or cnange d/n to float
if decimal, reduce from 4 to 5 noise
*/


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

// Serial2 pins of ESP32
#define RXD2 16
#define TXD2 17

//Pin Declaration
#define irrigationValve1Pin 12
#define irrigationValve2Pin 13
#define irrigationValve3Pin 4
#define irrigationValve4Pin 18
#define wellWaterPressureSensorPin 39
#define wellWaterValvePin 22
#define s0Pin 15 // s0-S3 select ADC channel
#define s1Pin 14 
#define s2Pin 5
#define s3Pin 0
#define currentSensorADCPin 36

//Irrigation variables
String message = "";
bool messageReady = false;
float soilHumidity;
int irrigationValve;
bool valve1;
bool valve2;
bool valve3;
bool valve4;
int wellWaterPressure = 0;
int wellWaterPressureLow = 0;
const int wellWaterPressureLowAtPeakHour = 15; // prevent well pump from running at peak hours by increasing cut on pressure (later add this to MAW demand control)
int wellWaterPressureHigh = 0;
bool cityWaterValveOn = false;
unsigned int wellPumpLastOff = 0;
int irrigationValveShed = 0;

float upTimeShed = 0;

// Current sensor variables
float calibrate[16] = {64.68, 30.95, 32.61, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // 0=well pump, 1-2 RV, 15.5A 2028W, 14.2A 1879
float avgSample = 1400;
float realTimeRMS[16];
float realTimeRMSSamples[16];
float hourRMS[16];
float hourRMSSamples[16];
bool isPeakHour;


Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

//prototype
void timer1sec();
void timer2sec();
void timer10sec();

Task task1sec(TASK_SECOND * 1, TASK_FOREVER, &timer1sec);
Task task2sec(TASK_SECOND * 2 , TASK_FOREVER, &timer2sec);
Task task10sec(TASK_SECOND * 10 , TASK_FOREVER, &timer10sec);

void getRMS(){
  esp_task_wdt_reset();
  for (int channel = 0; channel < 3; channel++){
//  int channel = 0; // only reading wellpump current for now, channel 0
    digitalWrite(s0Pin, bitRead(channel, 0));
    digitalWrite(s1Pin, bitRead(channel, 1));
    digitalWrite(s2Pin, bitRead(channel, 2));
    digitalWrite(s3Pin, bitRead(channel, 3));
    int sampleCount = 0;
    float readADC;
    float storeADC;
    float runningSum = 0;
    float runningSquare = 0;
    unsigned int sampleTime = micros();
    while (micros()<sampleTime+166667){ // sample 10 cycles x16.7ms
      readADC = (analogRead(currentSensorADCPin));
      storeADC = 0;
      if (abs(readADC-avgSample) >= 5.00) storeADC = abs(readADC-avgSample)-5.00; // remove noise
      runningSquare += storeADC*storeADC;
  //    if ((readADC-avgSample)*(readADC-avgSample) >=40.00) runningSquare += (readADC-avgSample)*(readADC-avgSample)-40.00; // remove noise
      runningSum += readADC;
  //    Serial.print("readADC-avgSample: ");
  //    Serial.print(readADC-avgSample);
      sampleCount++;
    }
    avgSample = runningSum/sampleCount;
    realTimeRMS[channel] += sqrt(runningSquare/sampleCount)*calibrate[channel];
    realTimeRMSSamples[channel]++;
    hourRMS[channel] += realTimeRMS[channel];
    hourRMSSamples[channel]++;
/*    Serial.print("channel: ");
    Serial.print(channel);
    Serial.print(", realTimeRMS: ");
    Serial.print(realTimeRMS[channel]/realTimeRMSSamples[channel]);
    Serial.print(", realTimeRMS[channel]: ");
    Serial.print(realTimeRMS[channel]);
    Serial.print(", realTimeRMSSamples[channel]: ");
    Serial.print(realTimeRMSSamples[channel]);
    Serial.print(", runningSquare/sampleCount: ");
    Serial.println(runningSquare/sampleCount);
*/    
  }
}

void turnCityWaterValveOnOff(){
  esp_task_wdt_reset();
  int wellWaterPressureSample = 0;
  for (int i=0; i < 10; i++){ // avg of 10 samples
    wellWaterPressureSample += analogRead(wellWaterPressureSensorPin);
  }
  wellWaterPressure = map(wellWaterPressureSample/10, 2500, 1200, 60, 30);
  if (wellWaterPressure <= wellWaterPressureLow+(isPeakHour*wellWaterPressureLowAtPeakHour)) cityWaterValveOn = true;
  if (wellPumpLastOff+(1000*60*10) < millis()) cityWaterValveOn = true; //turn on if pump has been running continously for 10min, likely low water
  if (wellWaterPressure >= wellWaterPressureHigh) cityWaterValveOn = false;
  digitalWrite(wellWaterValvePin, cityWaterValveOn);
/*  Serial.print("millis: ");
  Serial.println(millis());
  Serial.print("wellPumpLastOff: ");
  Serial.println(wellPumpLastOff);  
  Serial.print("millis()-(1000*60*10): ");
  Serial.println(millis()-(1000*60*10));
  Serial.print("cityWaterValveOn: ");
  Serial.println(cityWaterValveOn);
*/
}

void turnIrrigationValvesOnOff(){
  esp_task_wdt_reset();
  valve1=false;
  valve2=false;
  valve3=false;
  valve4=false;
  switch(irrigationValve){
    case 1:
    valve1=true;
    break;
    case 2:
    valve2=true;
    break;
    case 3:
    valve3=true;
    break;
    case 4:
    valve4=true;
    break;
  }
  digitalWrite(irrigationValve1Pin, valve1);
  digitalWrite(irrigationValve2Pin, valve2);
  digitalWrite(irrigationValve3Pin, valve3);
  digitalWrite(irrigationValve4Pin, valve4);
/*  Serial.print("irrigationValve1: ");
  Serial.println(valve1);
  Serial.print("irrigationValve2: ");
  Serial.println(valve2);
  Serial.print("irrigationValve3: ");
  Serial.println(valve3);
  Serial.print("irrigationValve4: ");
  Serial.println(valve4);
  */
}

void sendMessageBlynk(){
  esp_task_wdt_reset();
  DynamicJsonDocument doc(1024);
  doc["soilHumidity"] = soilHumidity;
  doc["cityWaterValveOn"] = cityWaterValveOn;
  doc["wellWaterPressure"] = wellWaterPressure;
  doc["upTimeShed"] = upTimeShed;
  doc["upTimeUART"] = millis()/1000.00/60.00/60.00;
  doc["realTimeChannel0kW"] = realTimeRMS[0]/realTimeRMSSamples[0];
  doc["realTimeChannel1kW"] = realTimeRMS[1]/realTimeRMSSamples[1];
  doc["realTimeChannel2kW"] = realTimeRMS[2]/realTimeRMSSamples[2];
  doc["irrigationValveShed"] = irrigationValveShed; // checking to see what is returned from shed
  serializeJson(doc, Serial2);
  String msg ;
  serializeJson(doc, msg);
  Serial.println("Message sent to Blynk board: " + msg);
}

void receiveMessageBlynk(){
  esp_task_wdt_reset();
  while (Serial2.available()){
    esp_task_wdt_reset();
    message = Serial2.readString();
    messageReady = true;
    Serial.print("Received message from Blynk board: "); Serial.println(message);
  }
  if (messageReady){
    DynamicJsonDocument doc(1024); // ArduinoJson version 6+
    DeserializationError error = deserializeJson(doc, message);
    if (error){
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      messageReady = false;
      return;
    }
    irrigationValve = doc["IrrigationValve"];
    wellWaterPressureLow = doc["wellWaterPressureLow"];
    wellWaterPressureHigh = doc["wellWaterPressureHigh"];
    isPeakHour = doc["isPeakHour"];
    messageReady = false;
    sendMessageBlynk ();
  }
} 

void sendMessageMesh(){
  esp_task_wdt_reset();
  // Serializing in JSON Format
  DynamicJsonDocument doc(1024);
  doc["IrrigationValve"] = irrigationValve;
  String msg ;
  serializeJson(doc, msg);
  mesh.sendBroadcast(msg);
  Serial.println("Message sent to mesh: " + msg);
}

// Message received from Mesh
void receivedCallback( uint32_t from, String &msg ){
  esp_task_wdt_reset();
  //Deserializing
  String json;
  DynamicJsonDocument doc(1024);
  json = msg.c_str();
  Serial.println("Received message from mesh: " + msg + " From: " + from);// + " Json: " + json);
  DeserializationError error = deserializeJson(doc, json);
  
  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
  }

  soilHumidity = doc["soilHumidity"];
  upTimeShed = doc["upTimeShed"];
  irrigationValveShed = doc["irrigationValveShed"]; // checking to see what is returned from shed
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
}

void timer2sec(){
  esp_task_wdt_reset();
  turnIrrigationValvesOnOff();
  sendMessageMesh();
  if (realTimeRMS[0]/realTimeRMSSamples[0] < 1000) wellPumpLastOff = millis(); //reset wellPumpLastOff if well pump is off
  for (int channel = 0; channel < 3; channel++){
    realTimeRMS[channel] = 0;
    realTimeRMSSamples[channel] = 0;
  }  
}

void timer10sec(){
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // For sending data to another ESP32
  Serial2.setTimeout(50);

  pinMode(irrigationValve1Pin, OUTPUT);
  pinMode(irrigationValve2Pin, OUTPUT);
  pinMode(irrigationValve3Pin, OUTPUT);
  pinMode(irrigationValve4Pin, OUTPUT);
  pinMode(wellWaterValvePin, OUTPUT);
  pinMode(s0Pin, OUTPUT);
  pinMode(s1Pin, OUTPUT);
  pinMode(s2Pin, OUTPUT);
  pinMode(s3Pin, OUTPUT);
    
  
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
  getRMS();
  receiveMessageBlynk();
  turnCityWaterValveOnOff();
  mesh.update();
}