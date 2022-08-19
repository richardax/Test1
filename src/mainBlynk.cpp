/*
Upgrades:
No water if rain coming day
https://api.openweathermap.org/data/2.5/forecast?id=4456595&appid=31dd74f53054210735646502e79f0305
"pop":0.5,"rain":{"3h":4.84} (0.5=50%, 4.84mm)
soak hose = 20mm/h


*/




#include <Arduino.h>
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 100 //x+10 sec watchdog timeout
//#include <WiFi.h>
//#include <WiFiClient.h>
//#include <BlynkSimpleEsp32.h>
//#include <WidgetRTC.h>
//WidgetRTC rtc;
#include <ArduinoJson.h>
#include <TimeLib.h> 


#define APP_DEBUG
#define BLYNK_PRINT Serial
#define USE_NODE_MCU_BOARD
#define BLYNK_FIRMWARE_VERSION "0.0.6"
#define BLYNK_TEMPLATE_ID "TMPLAtcxMrjL"
#define BLYNK_DEVICE_NAME "Smart Home Mesh"
//#define BLYNK_AUTH_TOKEN "w4Uqv4WIV7OM1TlBADRt6ekhP3nw60y-"
//char auth[] =  "0_skQseUAVEcETOvW29FJnhJmxraX_fF";
//char auth[] = "PDnkHUNH8grDqYhmLBJGkPU1DJ0W6Twk"; //old Blynk
#include "BlynkEdgent.h"
//BlynkTimer timer;
//unsigned int lastTimeCheckBlynkConnected = 0;
//const int blynkTimeout = 4000; //timeout = blynkTimeout*3ms
// WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "Richard-Joy_N";
//char pass[] = "richardjoyax";
//char auth[] = BLYNK_AUTH_TOKEN;

BLYNK_CONNECTED() {
  Blynk.syncAll();
}

// Serial2 pins of ESP32
#define RXD2 16
#define TXD2 17


// Peak demand Variables
const byte summerPeakStartMonth = 6; // summer starts June 1'st
const byte winterPeakStartMonth = 10; // winter starts Oct 1'st
const byte summerPeakStartHour = 13; 
const byte summerPeakEndHour = 19;
const byte winterPeakStartHour = 7;
const byte winterPeakEndHour = 12;
const byte peakWeekdayStart = 2; // mon
const byte peakWeekdayEnd = 6; // fri
bool isPeakHour = true;
bool summerTime = true;

// power variables
float realTimeChannel0kW = 0; //Well pump
float realTimeChannel1kW = 0; //RV1
float realTimeChannel2kW = 0; //RV2

// Irrigation variables
bool irrigationOn = false;
int irrigationStartHour = 0;
int irrigationTimesPerDay = 0;
int irrigationMinutesPerTime = 0;
int minutesSinceIrrigationStart = 0;
int irrigationValve = 0;
int irrigationValveShed = 0;
int forceValve = 0;
float soilHumidity = 0;
int wellWaterPressure = 0;
int wellWaterPressureLow = 0;
int wellWaterPressureHigh = 0;
bool cityWaterValveOn = false;
int noWaterIfSHHigherThan = 0;
float avgSoilHumidity = 0;
int alarmIfSHLowerThan = 0;
const int avgSoilHumidityMAmin = 1440; //number of minutes for moving average of soilhumidity 
float soilHumidityArray[avgSoilHumidityMAmin];

String message = "";
bool messageReady = false;

long unixTime;
float upTimeUART;
float upTimeShed;
unsigned int timeReceiveUART;
unsigned int timeSendtoUART;
unsigned int cityWaterValveLastOff = 0;

// Data Coming from Blynk App


BLYNK_WRITE(InternalPinRTC){   //check the value of InternalPinRTC  
  setTime(param.asLong());      //store time in t variable
  //Serial.print("Unix time: ");  
  //Serial.print(unixTime);              //prints time in UNIX format to Serial Monitor
  //Serial.println();
  }

BLYNK_WRITE(V0){ // irrigation on/off
  irrigationOn = param.asInt();
  Serial.print("irrigationOn: ");
  Serial.println(irrigationOn);
}

BLYNK_WRITE(V1){ // irrigation start hour
  irrigationStartHour = param.asInt();
  Serial.print("irrigationStartHour: ");
  Serial.println(irrigationStartHour);
}

BLYNK_WRITE(V2){ // irrigation how many times per day
  irrigationTimesPerDay = param.asInt();
  Serial.print("irrigationTimesPerDay: ");
  Serial.println(irrigationTimesPerDay);
}

BLYNK_WRITE(V3){ // minutes per time
  irrigationMinutesPerTime = param.asInt();
  Serial.print("irrigationMinutesPerTime: ");
  Serial.println(irrigationMinutesPerTime);
  }

BLYNK_WRITE(V4){ // wellWaterPressureLow
  wellWaterPressureLow = param.asInt();
  Serial.print("wellWaterPressureLow: ");
  Serial.println(wellWaterPressureLow);
  }

BLYNK_WRITE(V5){ // wellWaterPressureHigh
  wellWaterPressureHigh = param.asInt();
  Serial.print("wellWaterPressureHigh: ");
  Serial.println(wellWaterPressureHigh);
  }

BLYNK_WRITE(V6){ // noWaterIfSHHigherThan
  noWaterIfSHHigherThan = param.asInt();
  Serial.print("noWaterIfSHHigherThan: ");
  Serial.println(noWaterIfSHHigherThan);
  }

BLYNK_WRITE(V7){ // alarmIfSHLowerThan
  alarmIfSHLowerThan = param.asInt();
  Serial.print("alarmIfSHLowerThan: ");
  Serial.println(alarmIfSHLowerThan);
  }

BLYNK_WRITE(V8){ // forceValve
  forceValve = param.asInt();
  Serial.print("forceValve: ");
  Serial.println(forceValve);
  }

// prototype
void timer1sec();
void timer2sec();
void timer5sec();
void timer1min();
void sendMessageToUART();
void printDigits(byte digits);
void checkIfPeakHour();
void calculateavgSoilHumidity();

void setIrrigationOnOff(){
  esp_task_wdt_reset();
  if (irrigationStartHour > hour(now())){
    minutesSinceIrrigationStart = ((hour(now())+24)*60)+minute(now())-irrigationStartHour*60;
  }
  else{
    minutesSinceIrrigationStart = (hour(now())*60)+minute(now())-irrigationStartHour*60;
  }
  Serial.print("minutesSinceIrrigationStart: ");
  Serial.println(minutesSinceIrrigationStart);
  irrigationValve = 0;
  if (irrigationOn==true){
     switch(irrigationTimesPerDay){
      case 1:
        if (minutesSinceIrrigationStart >= 0 && minutesSinceIrrigationStart < irrigationMinutesPerTime*8){
          irrigationValve = ((minutesSinceIrrigationStart/irrigationMinutesPerTime)+1);
        }
        break;
      case 2:
        if (minutesSinceIrrigationStart >= 0 && minutesSinceIrrigationStart < irrigationMinutesPerTime*8){
          irrigationValve = ((minutesSinceIrrigationStart/irrigationMinutesPerTime)+1);
        }
        else{
          if (minutesSinceIrrigationStart >= 720 && minutesSinceIrrigationStart < 720+(irrigationMinutesPerTime*8)){
            irrigationValve = (((minutesSinceIrrigationStart-720)/irrigationMinutesPerTime)+1);
          }
        }
        break;
      case 3:
        if (minutesSinceIrrigationStart >= 0 && minutesSinceIrrigationStart < irrigationMinutesPerTime*8){
          irrigationValve = ((minutesSinceIrrigationStart/irrigationMinutesPerTime)+1);
        }
        else{
          if (minutesSinceIrrigationStart >= 480 && minutesSinceIrrigationStart < 480+(irrigationMinutesPerTime*8)){
            irrigationValve = (((minutesSinceIrrigationStart-480)/irrigationMinutesPerTime)+1);
          }
          else{
            if (minutesSinceIrrigationStart >= 960 && minutesSinceIrrigationStart < 960+(irrigationMinutesPerTime*8)){
              irrigationValve = (((minutesSinceIrrigationStart-960)/irrigationMinutesPerTime)+1);
            }
          }
        }
        break;
      case 4:
        if (minutesSinceIrrigationStart >= 0 && minutesSinceIrrigationStart < irrigationMinutesPerTime*8){
          irrigationValve = ((minutesSinceIrrigationStart/irrigationMinutesPerTime)+1);
        }
        else{
          if (minutesSinceIrrigationStart >= 360 && minutesSinceIrrigationStart < 360+(irrigationMinutesPerTime*8)){
            irrigationValve = (((minutesSinceIrrigationStart-360)/irrigationMinutesPerTime)+1);
          }
          else{
            if (minutesSinceIrrigationStart >= 720 && minutesSinceIrrigationStart < 720+(irrigationMinutesPerTime*8)){
              irrigationValve = (((minutesSinceIrrigationStart-720)/irrigationMinutesPerTime)+1);
            }
            else{
              if (minutesSinceIrrigationStart >= 1080 && minutesSinceIrrigationStart < 1080+(irrigationMinutesPerTime*8)){
                irrigationValve = (((minutesSinceIrrigationStart-1080)/irrigationMinutesPerTime)+1);
              }
            }
          }
        }
        break;
    }
  }
  if (avgSoilHumidity > noWaterIfSHHigherThan) irrigationValve = 0;
  if (forceValve > 0) irrigationValve = forceValve;
}

void timer1sec(){
  esp_task_wdt_reset();
  if(millis()/1000 < avgSoilHumidityMAmin) calculateavgSoilHumidity(); // prefill array when board starts
}

void timer2sec(){
  Serial.print("RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.print(" WiFi.status(): ");
  Serial.print(WiFi.status());
  Serial.print(" IP: ");
  Serial.print(WiFi.localIP());
  Serial.print(" Blynk.connected(): ");
  Serial.println(Blynk.connected());
  
  esp_task_wdt_reset();
  setIrrigationOnOff();
  sendMessageToUART();
  if (!cityWaterValveOn) cityWaterValveLastOff = millis(); 
  Blynk.virtualWrite(V120, realTimeChannel0kW);
  Blynk.virtualWrite(V121, realTimeChannel1kW);
  Blynk.virtualWrite(V122, realTimeChannel2kW);
}

void timer10sec(){
  esp_task_wdt_reset();
  checkIfPeakHour();
  Blynk.virtualWrite(V100, soilHumidity);
  Blynk.virtualWrite(V101, irrigationValve);
  Blynk.virtualWrite(V102, irrigationValveShed);
  Blynk.virtualWrite(V103, cityWaterValveOn*255);
  Blynk.virtualWrite(V104, wellWaterPressure);
  Blynk.virtualWrite(V105, millis()/1000.00/60.00/60.00);
  Blynk.virtualWrite(V106, upTimeUART);
  Blynk.virtualWrite(V110, upTimeShed);
  Blynk.virtualWrite(V111, WiFi.RSSI());
  Blynk.virtualWrite(V112, avgSoilHumidity);
}

void timer1min(){
  esp_task_wdt_reset();
  calculateavgSoilHumidity();
  if (avgSoilHumidity < alarmIfSHLowerThan) Blynk.logEvent("soil_dry");
  if (cityWaterValveLastOff + 15*60000 < millis()) Blynk.logEvent("city_water_on_15_min");
 }

void calculateavgSoilHumidity(){
  esp_task_wdt_reset();
  float soilHumiditySum = 0;
  for(int i=0;i<avgSoilHumidityMAmin-1;i++){ // move stored values to the previous record 
    soilHumidityArray[i] = soilHumidityArray[i+1];
    soilHumiditySum += soilHumidityArray[i];
  }
  soilHumidityArray[avgSoilHumidityMAmin-1] = soilHumidity;
  soilHumiditySum += soilHumidity;
  avgSoilHumidity = soilHumiditySum / avgSoilHumidityMAmin;
  }

void sendMessageToUART(){ //~2ms @doc(1024)
  esp_task_wdt_reset();
//  unsigned int timeSendtoUARTstart = micros();
  DynamicJsonDocument doc(1024);
  doc["IrrigationValve"] = irrigationValve; 
  doc["wellWaterPressureLow"] = wellWaterPressureLow;
  doc["wellWaterPressureHigh"] = wellWaterPressureHigh;
  doc["isPeakHour"] = isPeakHour;
  serializeJson(doc, Serial2); // Sending data to UART ESP32
  String msg ;
  serializeJson(doc, msg);
  Serial.println("Message sent to UART: " + msg);
  
//  timeSendtoUART = micros() - timeSendtoUARTstart;
}

void receiveMessageUART(){ //~2ms @doc(1024)
  esp_task_wdt_reset();
  unsigned int startReading = millis() + 1000;
  while (Serial2.available() && millis() < startReading){
    esp_task_wdt_reset();
    message = Serial2.readString();
    messageReady = true;
    Serial.print("Received message from UART: "); Serial.println(message);
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
    soilHumidity = doc["soilHumidity"];
    cityWaterValveOn = doc["cityWaterValveOn"];
    wellWaterPressure = doc["wellWaterPressure"];
    upTimeShed = doc["upTimeShed"];
    upTimeUART = doc["upTimeUART"];
    realTimeChannel0kW = doc["realTimeChannel0kW"];
    realTimeChannel1kW = doc["realTimeChannel1kW"];
    realTimeChannel2kW = doc["realTimeChannel2kW"];
    irrigationValveShed = doc["irrigationValveShed"]; // checking to see what is returned from shed
    messageReady = false;
  }
}   

void checkIfPeakHour(){ // check time and set isPeakHour = true/false
  esp_task_wdt_reset();
  Serial.println("\nChecking if isPeakHour");
  Blynk.sendInternal("rtc", "sync");
  long timeNow = now();
  int yearNow = year(timeNow);
  byte monthNow = month(timeNow);
  byte dayNow = day(timeNow);
  byte weekdayNow = weekday(timeNow); // 1=Sun 7=Sat
  byte hourNow = hour(timeNow);
  byte minuteNow = minute(timeNow);
  byte secondNow = second(timeNow);

  // DST for USA
  summerTime = false;
  int lastSunday = dayNow-weekdayNow;
  if ((monthNow  >  3 && monthNow < 11 ) ||                                 // DST always Apr-Oct
      (monthNow ==  3 && dayNow >= 8 && weekdayNow == 1 && hourNow >= 2) || // DST starts 2nd Sunday of March;  2am
      (monthNow ==  3 && lastSunday > 8) ||  
      (monthNow == 11 && dayNow <  8 && weekdayNow == 1 && hourNow <  2) || 
      (monthNow == 11 && lastSunday < 0)){                                 // DST ends 1st Sunday of November; 2am
      summerTime = true;
      }

 isPeakHour = false;
  if((weekdayNow >= peakWeekdayStart && weekdayNow <= peakWeekdayEnd) // if weekday AND ((summer AND summerPeakHour) OR (winter AND winterPeakHour))
    &&(
      ((monthNow >= summerPeakStartMonth && monthNow < winterPeakStartMonth) && (hourNow >= summerPeakStartHour && hourNow < summerPeakEndHour))
      ||
      (!(monthNow >= summerPeakStartMonth && monthNow < winterPeakStartMonth) && (hourNow >= winterPeakStartHour && hourNow < winterPeakEndHour))
      )){
    isPeakHour = true;
  }
  
  // digital clock display of current time
  Serial.print("Time: ");
  Serial.print(yearNow);
  Serial.print("-");
  printDigits(monthNow);
  Serial.print("-");
  printDigits(dayNow);
  Serial.print(" ");
  printDigits(hourNow);
  Serial.print(":");
  printDigits(minuteNow);
  Serial.print(":");
  printDigits(secondNow);
  Serial.print(" wd: ");
  Serial.print(weekdayNow); //weekday in numbers 1=sun, 7=Sat
  Serial.print(" lastSun: ");
  Serial.print(lastSunday); 
  Serial.print(" Summertime: ");
  Serial.print(summerTime); 
  Serial.print(" isPeakHour: ");
  Serial.print(isPeakHour);
  Serial.println();
}

void printDigits(byte digits){
  if(digits < 10){
    Serial.print('0');
  }
    Serial.print(digits,DEC);
}

/*
void initWiFi() {
  Serial.println("Connecting to Blynk...");
  Blynk.disconnect();
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //WiFi.mode(WIFI_STA);
  //WiFi.begin(ssid, pass);
  //Serial.println("Connecting to Blynk...");
  int connectionAttempts = 0;
  while (!Blynk.connected()) {
    connectionAttempts ++;
    Serial.print("Connecting to Blynk, attempt: ");
    Serial.println(connectionAttempts);
    if (connectionAttempts >30) break;
    esp_task_wdt_reset();
    timer.run(); // keep running tasks without internet/Blynk
    delay(1000);
  }
  if (Blynk.connected()){
    Serial.println("Blynk connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());  
  }
  else{
    Serial.println("Blynk did not connect");
  }
}
*/


void setup()
{
  Serial.begin(115200); // For Debugging purpose
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // For sending data to another ESP32
  Serial2.setTimeout(50);
  edgentTimer.setInterval(1000L, timer1sec);
  edgentTimer.setInterval(2000L, timer2sec);
  edgentTimer.setInterval(10000L, timer10sec);
  edgentTimer.setInterval(60000L, timer1min);
  //Serial.print("Connecting");
  //Blynk.begin(auth, ssid, pass);
  //Serial.print("Connected");
  delay(100);
  BlynkEdgent.begin();
//  initWiFi();
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  delay(1000);
 // Blynk.sendInternal("rtc", "sync");
}

void loop(){
  esp_task_wdt_reset();
  receiveMessageUART();
  /*if (!Blynk.connected() && lastTimeCheckBlynkConnected < millis()-60000){
    Serial.print("Blynk Disconnected");
    lastTimeCheckBlynkConnected = millis();
    initWiFi();
  }
  if (Blynk.connected()){
    lastTimeCheckBlynkConnected = millis();
    Blynk.run();
  }
  */
 /*
  if(Blynk.connected()) BlynkEdgent.run();
  if(!Blynk.connected()){
    Serial.print("Blynk disconnected");
    delay(500);
//    Blynk.disconnect();
//    BlynkEdgent.begin();
  }
  */
  
  BlynkEdgent.run();
 // timer.run();
}